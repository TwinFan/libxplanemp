# libxplanemp

## Deprecation Note

**Note: This fork will no longer be maintained any further!**
**Check out [TwinFan/XPMP2](https://github.com/TwinFan/XPMP2) instead,** which is **ready for Vulkan/Metal** by implementing **[instancing](https://developer.x-plane.com/sdk/XPLMInstance/)**, and also has **[TCAS Override](https://developer.x-plane.com/sdk/XPLMInstance/)** already: up to 63 TCAS blibs without configuring AI Aircraft.

XPMP2 can be used as a **replacement for libxplanemp** with only very little source code changes. **New projects shall definitely base on XPMP2!**

## Multiplayer library for X-Plane

This is libxplanemp, the multiplayer client code for X-Plane.

Changes in this TwinFan fork as compared to the official kuroneko fork are summarized in the [Wiki](https://github.com/TwinFan/libxplanemp/wiki).

See README.MULTIPLAYER for the original README file.

### Additional Notes

* Included CMakeFiles can be integrated into your project directly

* Make sure you define `XPMP_CLIENT_NAME` and `XPMP_CLIENT_LONGNAME` to
  reflect your client/plugin.

## License
```
Copyright (c) 2006-2013, Ben Supnik and Chris Serio
Copyright (c) 2015-2017, Christopher Collins
Copyright (c) 2016-2017, Roland Winklmeier & Matthew Sutcliffe

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notices and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```
