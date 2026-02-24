/**
* Headless DM-VIO dataset runner that incrementally writes a point cloud to PLY.
*
* It follows the same data flow as main_dmvio_dataset.cpp, but adds a custom
* Output3DWrapper to extract sparse points from keyframes and save them
* repeatedly so Ctrl+C still leaves a recent point cloud on disk.
*/

#include "util/MainSettings.h"

#include <atomic>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <locale.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/thread.hpp>

#include "dso/util/settings.h"
#include "dso/util/globalFuncs.h"
#include "dso/util/DatasetReader.h"
#include "dso/util/globalCalib.h"
#include "util/TimeMeasurement.h"

#include "dso/util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"

#include "IOWrapper/Output3DWrapper.h"

#include <util/SettingsUtil.h>

using namespace dso;

namespace
{
std::atomic<bool> gStopRequested(false);

void handleSignal(int)
{
    gStopRequested.store(true);
}

struct PointXYZ
{
    float x;
    float y;
    float z;
};

class IncrementalPlyOutputWrapper : public IOWrap::Output3DWrapper
{
public:
    IncrementalPlyOutputWrapper(std::string plyPath, int saveEveryNCallbacks)
            : plyPath_(std::move(plyPath)), saveEveryNCallbacks_(saveEveryNCallbacks)
    {
        if(saveEveryNCallbacks_ < 1) saveEveryNCallbacks_ = 1;
    }

    void publishKeyframes(std::vector<FrameHessian*>& frames, bool /*final*/, CalibHessian* HCalib) override
    {
        if(HCalib == nullptr) return;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            for(FrameHessian* frame : frames)
            {
                if(frame == nullptr || frame->shell == nullptr) continue;
                frameCloud_[frame->frameID] = extractFramePoints(frame, HCalib);
            }

            ++numKeyframeCallbacks_;
            if(numKeyframeCallbacks_ % saveEveryNCallbacks_ == 0)
            {
                saveToPlyLocked();
            }
        }
    }

    void join() override
    {
        forceSave();
    }

    void reset() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frameCloud_.clear();
        numKeyframeCallbacks_ = 0;
        saveToPlyLocked();
    }

    void forceSave()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        saveToPlyLocked();
    }

private:
    std::vector<PointXYZ> extractFramePoints(FrameHessian* frame, CalibHessian* HCalib)
    {
        std::vector<PointXYZ> points;
        if(frame == nullptr || frame->shell == nullptr || HCalib == nullptr) return points;

        const double fx = HCalib->fxl();
        const double fy = HCalib->fyl();
        const double cx = HCalib->cxl();
        const double cy = HCalib->cyl();
        if(!(fx > 0.0) || !(fy > 0.0)) return points;

        points.reserve(frame->pointHessians.size() + frame->pointHessiansMarginalized.size());

        auto appendPoints = [&](const std::vector<PointHessian*>& src)
        {
            for(PointHessian* p : src)
            {
                if(p == nullptr) continue;
                if(p->status == PointHessian::OUTLIER || p->status == PointHessian::OOB) continue;

                const double idepth = p->idepth_scaled;
                if(!(idepth > 1e-9) || !std::isfinite(idepth)) continue;

                const double z = 1.0 / idepth;
                const double x = (p->u - cx) * z / fx;
                const double y = (p->v - cy) * z / fy;

                const Vec3 pCam(x, y, z);
                const Vec3 pWorld = frame->shell->camToWorld * pCam;
                if(!std::isfinite(pWorld.x()) || !std::isfinite(pWorld.y()) || !std::isfinite(pWorld.z())) continue;

                points.push_back(PointXYZ{
                        static_cast<float>(pWorld.x()),
                        static_cast<float>(pWorld.y()),
                        static_cast<float>(pWorld.z())
                });
            }
        };

        appendPoints(frame->pointHessians);
        appendPoints(frame->pointHessiansMarginalized);
        return points;
    }

    void saveToPlyLocked()
    {
        std::size_t totalPoints = 0;
        for(const auto& kv : frameCloud_) totalPoints += kv.second.size();

        const std::string tmpPath = plyPath_ + ".tmp";
        std::ofstream out(tmpPath, std::ios::trunc | std::ios::out);
        if(!out.good())
        {
            std::cerr << "WARNING: failed to open PLY output: " << tmpPath << std::endl;
            return;
        }

        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "comment dmvio incremental sparse cloud\n";
        out << "element vertex " << totalPoints << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "end_header\n";

        out << std::fixed << std::setprecision(6);
        for(const auto& kv : frameCloud_)
        {
            for(const PointXYZ& p : kv.second)
            {
                out << p.x << " " << p.y << " " << p.z << "\n";
            }
        }
        out.close();

        if(std::rename(tmpPath.c_str(), plyPath_.c_str()) != 0)
        {
            std::cerr << "WARNING: failed to move " << tmpPath << " to " << plyPath_ << std::endl;
        }
    }

private:
    std::string plyPath_;
    int saveEveryNCallbacks_;
    std::size_t numKeyframeCallbacks_ = 0;

    std::unordered_map<int, std::vector<PointXYZ>> frameCloud_;
    std::mutex mutex_;
};
} // namespace


std::string gtFile = "";
std::string tsFile = "";
std::string source = "";
std::string imuFile = "";
std::string plyFile = "";

bool reverse = false;
int start = 0;
int end = 100000;
int maxPreloadImages = 0;
int plySaveEveryN = 1;
bool use16Bit = false;

dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;

void run(ImageFolderReader* reader)
{
    if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == nullptr)
    {
        std::cerr << "ERROR: missing photometric calibration. Use mode=1 or pass gamma/vignette files." << std::endl;
        return;
    }

    int lstart = start;
    int lend = end;
    int linc = 1;
    if(reverse)
    {
        if(setting_useIMU)
        {
            std::cerr << "ERROR: reverse playback is not supported with IMU." << std::endl;
            return;
        }
        lstart = end - 1;
        if(lstart >= reader->getNumImages()) lstart = reader->getNumImages() - 1;
        lend = start;
        linc = -1;
    }

    if(mainSettings.preload && maxPreloadImages > 0 && reader->getNumImages() > maxPreloadImages)
    {
        std::cout << "maxPreloadImages exceeded; disabling preload.\n";
        mainSettings.preload = false;
    }

    std::vector<int> idsToPlay;
    idsToPlay.reserve(reader->getNumImages());
    for(int i = lstart; i >= 0 && i < reader->getNumImages() && linc * i < linc * lend; i += linc)
    {
        idsToPlay.push_back(i);
    }

    std::vector<ImageAndExposure*> preloadedImages;
    if(mainSettings.preload)
    {
        std::cout << "LOADING ALL IMAGES!\n";
        preloadedImages.reserve(idsToPlay.size());
        for(int id : idsToPlay)
        {
            preloadedImages.push_back(reader->getImage(id));
        }
    }

    const bool linearizeOperation = (mainSettings.playbackSpeed == 0);
    auto* fullSystem = new FullSystem(linearizeOperation, imuCalibration, imuSettings);
    fullSystem->setGammaFunction(reader->getPhotometricGamma());

    if(plyFile.empty())
    {
        plyFile = imuSettings.resultsPrefix + "pointcloud_incremental.ply";
    }
    auto plyWrapper = std::make_unique<IncrementalPlyOutputWrapper>(plyFile, plySaveEveryN);
    fullSystem->outputWrapper.push_back(plyWrapper.get());

    const bool gtDataThere = reader->loadGTData(gtFile);

    dmvio::IMUData skippedIMUData;
    bool imuDataSkipped = false;
    for(std::size_t ii = 0; ii < idsToPlay.size(); ++ii)
    {
        if(gStopRequested.load()) break;

        const int id = idsToPlay[ii];
        ImageAndExposure* img = mainSettings.preload ? preloadedImages[ii] : reader->getImage(id);

        dmvio::GTData data;
        bool gtFound = false;
        if(gtDataThere)
        {
            data = reader->getGTData(id, gtFound);
        }

        std::unique_ptr<dmvio::IMUData> imuData;
        if(setting_useIMU)
        {
            imuData = std::make_unique<dmvio::IMUData>(reader->getIMUData(id));
        }

        if(imuDataSkipped && imuData)
        {
            imuData->insert(imuData->begin(), skippedIMUData.begin(), skippedIMUData.end());
            skippedIMUData.clear();
            imuDataSkipped = false;
        }

        fullSystem->addActiveFrame(img, id, imuData.get(), (gtDataThere && gtFound) ? &data : nullptr);
        delete img;

        if(fullSystem->initFailed || setting_fullResetRequested)
        {
            std::cout << "RESETTING!\n";
            auto wraps = fullSystem->outputWrapper;
            delete fullSystem;
            for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

            fullSystem = new FullSystem(linearizeOperation, imuCalibration, imuSettings);
            fullSystem->setGammaFunction(reader->getPhotometricGamma());
            fullSystem->outputWrapper = wraps;
            setting_fullResetRequested = false;
        }

        if(fullSystem->isLost)
        {
            std::cout << "LOST!!\n";
            break;
        }
    }

    fullSystem->blockUntilMappingIsFinished();
    plyWrapper->forceSave();

    fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);
    fullSystem->printResult(imuSettings.resultsPrefix + "resultKFs.txt", true, false, false);
    fullSystem->printResult(imuSettings.resultsPrefix + "resultScaled.txt", false, true, true);
    dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper) ow->join();

    delete fullSystem;
    delete reader;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "C");

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("files", source);
    settingsUtil->registerArg("start", start);
    settingsUtil->registerArg("end", end);
    settingsUtil->registerArg("imuFile", imuFile);
    settingsUtil->registerArg("gtFile", gtFile);
    settingsUtil->registerArg("tsFile", tsFile);
    settingsUtil->registerArg("reverse", reverse);
    settingsUtil->registerArg("use16Bit", use16Bit);
    settingsUtil->registerArg("maxPreloadImages", maxPreloadImages);
    settingsUtil->registerArg("plyFile", plyFile);
    settingsUtil->registerArg("plySaveEveryN", plySaveEveryN);

    mainSettings.parseArguments(argc, argv, *settingsUtil);

    if(mainSettings.imuCalibFile != "")
    {
        imuCalibration.loadFromFile(mainSettings.imuCalibFile);
    }

    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    auto* reader = new ImageFolderReader(source, mainSettings.calib, mainSettings.gammaCalib,
                                         mainSettings.vignette, use16Bit, tsFile);
    reader->loadIMUData(imuFile);
    reader->setGlobalCalibration();

    run(reader);
    return 0;
}

