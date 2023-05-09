#include "dev/power/g500.hh"

#include "sim/system.hh"

namespace gem5
{

using namespace std;

PowerISA::G500::G500(const Params &p)
    : Platform(p)
{}

void
PowerISA::G500::postConsoleInt()
{
    //warn_once("Don't know what interrupt to post for console.\n");
    cout<<"Post console intr\n";
    //this->intrctrl->post(2,0);
    //panic("Need implementation\n");
}

void
PowerISA::G500::clearConsoleInt()
{
    //warn_once("Don't know what interrupt to clear for console.\n");
    //cout<<"Clear console intr\n";
    //this->intrctrl->clear(2,0);
    //panic("Need implementation\n");
}

void
PowerISA::G500::postPciInt(int line)
{
    panic("Need implementation\n");
}

void
PowerISA::G500::clearPciInt(int line)
{
    panic("Need implementation\n");
}

void
PowerISA::G500::serialize(CheckpointOut &cp) const
{
}

void
PowerISA::G500::unserialize(CheckpointIn &cp)
{
}

}
