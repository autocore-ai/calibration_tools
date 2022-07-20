// CMake generated file. Do Not Edit.

#pragma once

namespace pangolin {

void RegisterNoneWindowFactory();
void RegisterX11WindowFactory();

inline bool LoadBuiltInWindowFrameworks()
{
    RegisterNoneWindowFactory();
    RegisterX11WindowFactory();
    return true;
}

} // pangolin
