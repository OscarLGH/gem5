 /**
 * @file
 * Declaration of top level class for the Gem5-power9. This class just
 * retains pointers to all its children so the children can communicate.
 *
 * Inspired from the SPARCH T1000 system.
 */

#ifndef __DEV_G500_HH__
#define __DEV_G500_HH__

#include "dev/platform.hh"
#include "params/G500.hh"

namespace gem5
{

namespace PowerISA
{

class G500 : public Platform
{

protected:
    System *system;

public:
    typedef G500Params Params;

    G500(const Params &p);
     /**
     * Cause the cpu to post a serial interrupt to the CPU.
     */
    void postConsoleInt() override;

    void clearConsoleInt() override;

    void postPciInt(int line) override;

    void clearPciInt(int line) override;

    void serialize(CheckpointOut &cp) const override;

    void unserialize(CheckpointIn &cp) override;
};

}
}

#endif
