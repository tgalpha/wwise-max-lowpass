#pragma once

#include "../max_lowpassPlugin.h"

class max_lowpassPluginGUI final
	: public AK::Wwise::Plugin::PluginMFCWindows<>
	, public AK::Wwise::Plugin::GUIWindows
{
public:
	max_lowpassPluginGUI();

};
