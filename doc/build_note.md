When building pangolin add

```
cmake -DCPP11_NO_BOOST=ON -DBUILD_PANGOLIN_FFMPEG=OFF -DCMAKE_CXX_FLAGS="-include cstdint -include cstdio" ..
```
