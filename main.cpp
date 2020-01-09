#include "settings.h"
#include "Eagle_Utils.h"
#include "getalignresults.h"

int main()
{
    Settings settings = Settings();
    EAGLE::checkPath(settings.keyFramesPath);
    getAlignResults align(settings);
    return 0;
}
