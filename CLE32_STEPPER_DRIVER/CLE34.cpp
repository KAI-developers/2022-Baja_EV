#include "CLE34.h"


CLE34::CLE34(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS)
    :   PUL_PLUS(PIN_PUL_PLUS),
        DIR_PLUS(PIN_DIR_PLUS),
        ENA_PLUS(PIN_ENA_PLUS)        
{
    float fDummy_sec = 1000.0;
    m_period_sec = fDummy_sec;

    time.start ();
    ticker_1ms.attach_us(&counter_us, 1);
}


void CLE34::counter_us()
{
    count_us++;
}


/*
dir CW or CCW
*/
void CLE34::setDir(int dir)
{
    DIR_PLUS = dir;
}
