#pragma once

#include <Core/CoreAll.h>
#include <sstream>

#define ms_format(x) ((std::ostringstream() << x).str())

using namespace adsk::core;

extern Ptr<Application> app;
extern Ptr<UserInterface> ui;

void error(const char* title, const char* msg);

void warn(const char* title, const char* msg);