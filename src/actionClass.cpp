#include "actionClass.h"

using namespace std;

// ******************** ACTION CLASS
actionStruct::actionStruct() {
        q = 0;
        actionStruct::tag = "UNKNOWN";
}


actionClass::actionClass()
{
    current_action = 0;
    current_status = ACTION_IDLE;
}

