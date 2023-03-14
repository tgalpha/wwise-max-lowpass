
#include "max_lowpassPluginGUI.h"

max_lowpassPluginGUI::max_lowpassPluginGUI()
{
}

ADD_AUDIOPLUGIN_CLASS_TO_CONTAINER(
    max_lowpass,            // Name of the plug-in container for this shared library
    max_lowpassPluginGUI,   // Authoring plug-in class to add to the plug-in container
    max_lowpassFX           // Corresponding Sound Engine plug-in class
);
