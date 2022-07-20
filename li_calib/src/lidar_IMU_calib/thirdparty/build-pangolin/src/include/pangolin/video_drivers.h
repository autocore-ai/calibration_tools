// CMake generated file. Do Not Edit.

#pragma once

namespace pangolin {

void RegisterTestVideoFactory();
void RegisterImagesVideoFactory();
void RegisterImagesVideoOutputFactory();
void RegisterSplitVideoFactory();
void RegisterTruncateVideoFactory();
void RegisterPvnVideoFactory();
void RegisterPangoVideoFactory();
void RegisterPangoVideoOutputFactory();
void RegisterDebayerVideoFactory();
void RegisterShiftVideoFactory();
void RegisterMirrorVideoFactory();
void RegisterUnpackVideoFactory();
void RegisterPackVideoFactory();
void RegisterJoinVideoFactory();
void RegisterMergeVideoFactory();
void RegisterJsonVideoFactory();
void RegisterThreadVideoFactory();
void RegisterFirewireVideoFactory();
void RegisterV4lVideoFactory();
void RegisterFfmpegVideoFactory();
void RegisterFfmpegVideoOutputFactory();
void RegisterOpenNiVideoFactory();
void RegisterOpenNi2VideoFactory();

inline bool LoadBuiltInVideoDrivers()
{
    RegisterTestVideoFactory();
    RegisterImagesVideoFactory();
    RegisterImagesVideoOutputFactory();
    RegisterSplitVideoFactory();
    RegisterTruncateVideoFactory();
    RegisterPvnVideoFactory();
    RegisterPangoVideoFactory();
    RegisterPangoVideoOutputFactory();
    RegisterDebayerVideoFactory();
    RegisterShiftVideoFactory();
    RegisterMirrorVideoFactory();
    RegisterUnpackVideoFactory();
    RegisterPackVideoFactory();
    RegisterJoinVideoFactory();
    RegisterMergeVideoFactory();
    RegisterJsonVideoFactory();
    RegisterThreadVideoFactory();
    RegisterFirewireVideoFactory();
    RegisterV4lVideoFactory();
    RegisterFfmpegVideoFactory();
    RegisterFfmpegVideoOutputFactory();
    RegisterOpenNiVideoFactory();
    RegisterOpenNi2VideoFactory();
    return true;
}

} // pangolin
