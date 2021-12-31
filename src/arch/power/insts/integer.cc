/*
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/power/insts/integer.hh"


bool inv_flag, lt_flag, gt_flag, eq_flag, ox_flag, sat_flag;              // wwf
bool vxsnan_flag, vximz_flag, vxidi_flag, vxisi_flag, vxzdz_flag, vxsqrt_flag, vxcvi_flag, vxvc_flag;
bool inc_flag, ux_flag, xx_flag, zx_flag;

namespace gem5
{

using namespace PowerISA;
using namespace std;

std::string
IntOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printDest = true;
    bool printSrcs = true;
    bool printSecondSrc = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "mtcrf" ||
        myMnemonic == "mtxer" ||
        myMnemonic == "mtlr"  ||
        myMnemonic == "mtctr" ||
        myMnemonic == "mttar" ||
        myMnemonic == "mttb"  ||
        myMnemonic == "mttbu") {
        printDest = false;
    } else if (myMnemonic == "mfcr"  ||
               myMnemonic == "mfxer" ||
               myMnemonic == "mflr"  ||
               myMnemonic == "mfctr" ||
               myMnemonic == "mftar" ||
               myMnemonic == "mftb"  ||
               myMnemonic == "mftbu") {
        printSrcs = false;
    }

    // Additional characters depending on isa bits being set
    if (oe)
        myMnemonic = myMnemonic + "o";
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0 && printDest)
        printReg(ss, destRegIdx(0));

    // Print the (possibly) two source registers
    if (_numSrcRegs > 0 && printSrcs) {
        if (_numDestRegs > 0 && printDest)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
        if (_numSrcRegs > 1 && printSecondSrc) {
          ss << ", ";
          printReg(ss, srcRegIdx(1));
        }
    }

    return ss.str();
}


std::string
IntImmOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
    }

    // Print the immediate value last
    ss << ", " << (int32_t)si;

    return ss.str();
}


std::string
IntArithOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSecondSrc = true;
    bool printThirdSrc = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "addme" ||
        myMnemonic == "addze" ||
        myMnemonic == "subfme" ||
        myMnemonic == "subfze" ||
        myMnemonic == "neg") {
        printSecondSrc = false;
    } else if (myMnemonic == "maddhd" ||
               myMnemonic == "maddhdu" ||
               myMnemonic == "maddld") {
        printThirdSrc = true;
    }

    // Additional characters depending on isa bits being set
    if (oe)
        myMnemonic = myMnemonic + "o";
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (_numSrcRegs > 1 && printSecondSrc) {
            ss << ", ";
            printReg(ss, srcRegIdx(1));

            // Print the third source register
            if (_numSrcRegs > 2 && printThirdSrc) {
                ss << ", ";
                printReg(ss, srcRegIdx(2));
            }
        }
    }

    return ss.str();
}


std::string
IntImmArithOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool negateImm = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "addi") {
        if (_numSrcRegs == 0) {
            myMnemonic = "li";
        } else if (si < 0) {
            myMnemonic = "subi";
            negateImm = true;
        }
    } else if (myMnemonic == "addis") {
        if (_numSrcRegs == 0) {
            myMnemonic = "lis";
        } else if (si < 0) {
            myMnemonic = "subis";
            negateImm = true;
        }
    } else if (myMnemonic == "addic" && si < 0) {
        myMnemonic = "subic";
        negateImm = true;
    } else if (myMnemonic == "addic_") {
        if (si < 0) {
            myMnemonic = "subic.";
            negateImm = true;
        } else {
            myMnemonic = "addic.";
        }
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
    }

    // Print the immediate value
    if (negateImm)
        ss << ", " << -si;
    else
        ss << ", " << si;

    return ss.str();
}


std::string
IntDispArithOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSrcs = true;
    bool printDisp = true;
    bool negateDisp = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "addpcis") {
        printSrcs = false;
        if (d == 0) {
            myMnemonic = "lnia";
            printDisp = false;
        } else if (d < 0) {
            myMnemonic = "subpcis";
            negateDisp = true;
        }
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the source register
    if (_numSrcRegs > 0 && printSrcs) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
    }

    // Print the displacement
    if (printDisp)
        ss << ", " << (negateDisp ? -d : d);

    return ss.str();
}


std::string
IntLogicOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSecondSrc = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "or" && srcRegIdx(0) == srcRegIdx(1)) {
        myMnemonic = "mr";
        printSecondSrc = false;
    } else if (myMnemonic == "extsb" ||
               myMnemonic == "extsh" ||
               myMnemonic == "extsw" ||
               myMnemonic == "cntlzw" ||
               myMnemonic == "cntlzd" ||
               myMnemonic == "cnttzw" ||
               myMnemonic == "cnttzd") {
        printSecondSrc = false;
    }

    // Additional characters depending on isa bits being set
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (printSecondSrc) {

            // If the instruction updates the CR, the destination register
            // Ra is read and thus, it becomes the second source register
            // due to its higher precedence over Rb. In this case, it must
            // be skipped.
            if (rc) {
                if (_numSrcRegs > 2) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(2));
                }
            } else {
                if (_numSrcRegs > 1) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(1));
                }
            }
        }
    }

    return ss.str();
}


std::string
IntImmLogicOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printRegs = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "ori" &&
        destRegIdx(0).index() == 0 && srcRegIdx(0).index() == 0) {
        myMnemonic = "nop";
        printRegs = false;
    } else if (myMnemonic == "xori" &&
               destRegIdx(0).index() == 0 && srcRegIdx(0).index() == 0) {
        myMnemonic = "xnop";
        printRegs = false;
    } else if (myMnemonic == "andi_") {
        myMnemonic = "andi.";
    } else if (myMnemonic == "andis_") {
        myMnemonic = "andis.";
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    if (printRegs) {

        // Print the first destination only
        if (_numDestRegs > 0)
            printReg(ss, destRegIdx(0));

        // Print the source register
        if (_numSrcRegs > 0) {
            if (_numDestRegs > 0)
                ss << ", ";
            printReg(ss, srcRegIdx(0));
        }

        // Print the immediate value
        ss << ", " << ui;
    }

     return ss.str();
}


std::string
IntCompOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printFieldPrefix = false;
    bool printLength = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "cmp" ||
        myMnemonic == "cmpl") {
        myMnemonic += l ? "d" : "w";
        printFieldPrefix = true;
        printLength = false;
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (printFieldPrefix) {
        if (bf > 0)
            ss << "cr" << (int) bf;
    } else {
        ss << (int) bf;
    }

    // Print the length
    if (printLength) {
        if (!printFieldPrefix || bf > 0)
            ss << ", ";
        ss << (int) l;
    }

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (!printFieldPrefix || bf > 0 || printLength)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (_numSrcRegs > 1) {
            ss << ", ";
            printReg(ss, srcRegIdx(1));
        }
    }

    return ss.str();
}


std::string
IntImmCompOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printFieldPrefix = false;
    bool printLength = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "cmpi") {
        myMnemonic = l ? "cmpdi" : "cmpwi";
        printFieldPrefix = true;
        printLength = false;
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (printFieldPrefix) {
        if (bf > 0)
            ss << "cr" << (int) bf;
    } else {
        ss << (int) bf;
    }

    // Print the length
    if (printLength) {
        if (!printFieldPrefix || bf > 0)
            ss << ", ";
        ss << (int) l;
    }

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (!printFieldPrefix || bf > 0 || printLength)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
    }

    // Print the immediate value
    ss << ", " << si;

    return ss.str();
}


std::string
IntImmCompLogicOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printFieldPrefix = false;
    bool printLength = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "cmpli") {
        myMnemonic = l ? "cmpldi" : "cmplwi";
        printFieldPrefix = true;
        printLength = false;
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (printFieldPrefix) {
        if (bf > 0)
            ss << "cr" << (int) bf;
    } else {
        ss << (int) bf;
    }

    // Print the length
    if (printLength) {
        if (!printFieldPrefix || bf > 0)
            ss << ", ";
        ss << (int) l;
    }

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (!printFieldPrefix || bf > 0 || printLength)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
    }

    // Print the immediate value
    ss << ", " << ui;

    return ss.str();
}


std::string
IntShiftOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSecondSrc = true;
    bool printShift = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "srawi") {
        printSecondSrc = false;
        printShift = true;
    }

    // Additional characters depending on isa bits being set
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (printSecondSrc) {

            // If the instruction updates the CR, the destination register
            // Ra is read and thus, it becomes the second source register
            // due to its higher precedence over Rb. In this case, it must
            // be skipped.
            if (rc) {
                if (_numSrcRegs > 2) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(2));
                }
            } else {
                if (_numSrcRegs > 1) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(1));
                }
            }
        }
    }

    // Print the shift value
    if (printShift)
        ss << ", " << (int) sh;

    return ss.str();
}


std::string
IntConcatShiftOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSecondSrc = true;
    bool printShift = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "sradi" ||
        myMnemonic == "extswsli") {
        printSecondSrc = false;
        printShift = true;
    }

    // Additional characters depending on isa bits being set
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (printSecondSrc) {

            // If the instruction updates the CR, the destination register
            // Ra is read and thus, it becomes the second source register
            // due to its higher precedence over Rb. In this case, it must
            // be skipped.
            if (rc) {
                if (_numSrcRegs > 2) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(2));
                }
            } else {
                if (_numSrcRegs > 1) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(1));
                }
            }
        }
    }

    // Print the shift value
    if (printShift)
        ss << ", " << (int) sh;

    return ss.str();
}


std::string
IntRotateOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSecondSrc = true;
    bool printShift = true;
    bool printMaskBeg = true;
    bool printMaskEnd = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "rlwinm") {
        if (mb == 0 && me == 31) {
            myMnemonic = "rotlwi";
            printMaskBeg = false;
            printMaskEnd = false;
        } else if (sh == 0 && me == 31) {
            myMnemonic = "clrlwi";
            printShift = false;
            printMaskEnd = false;
        }
        printSecondSrc = false;
    } else if (myMnemonic == "rlwnm") {
        if (mb == 0 && me == 31) {
            myMnemonic = "rotlw";
            printMaskBeg = false;
            printMaskEnd = false;
        }
        printShift = false;
    } else if (myMnemonic == "rlwimi") {
        printSecondSrc = false;
    }

    // Additional characters depending on isa bits being set
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (printSecondSrc) {

            // If the instruction updates the CR, the destination register
            // Ra is read and thus, it becomes the second source register
            // due to its higher precedence over Rb. In this case, it must
            // be skipped.
            if (rc) {
                if (_numSrcRegs > 2) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(2));
                }
            } else {
                if (_numSrcRegs > 1) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(1));
                }
            }
        }
    }

    // Print the shift value
    if (printShift)
        ss << ", " << (int) sh;

    // Print the mask bounds
    if (printMaskBeg)
        ss << ", " << (int) mb;
    if (printMaskEnd)
        ss << ", " << (int) me;

    return ss.str();
}

std::string
IntConcatRotateOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    bool printSecondSrc = false;
    bool printShift = true;
    bool printMaskBeg = true;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "rldicl") {
        if (mb == 0) {
            myMnemonic = "rotldi";
            printMaskBeg = false;
        } else if (sh == 0) {
            myMnemonic = "clrldi";
            printShift = false;
        }
    } else if (myMnemonic == "rldcl") {
        if (mb == 0) {
            myMnemonic = "rotld";
            printMaskBeg = false;
        }
        printSecondSrc = true;
        printShift = false;
    } else if (myMnemonic == "rldcr") {
        printSecondSrc = true;
        printShift = false;
    }

    // Additional characters depending on isa bits being set
    if (rc)
        myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0)
        printReg(ss, destRegIdx(0));

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0)
            ss << ", ";
        printReg(ss, srcRegIdx(0));

        // Print the second source register
        if (printSecondSrc) {

            // If the instruction updates the CR, the destination register
            // Ra is read and thus, it becomes the second source register
            // due to its higher precedence over Rb. In this case, it must
            // be skipped.
            if (rc) {
                if (_numSrcRegs > 2) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(2));
                }
            } else {
                if (_numSrcRegs > 1) {
                    ss << ", ";
                    printReg(ss, srcRegIdx(1));
                }
            }
        }
    }

    // Print the shift amount
    if (printShift)
        ss << ", " << (int) sh;

    // Print the mask bound
    if (printMaskBeg)
        ss << ", " << (int) mb;

    return ss.str();
}


std::string
IntTrapOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::string ext;
    std::stringstream ss;
    bool printSrcs = true;
    bool printCond = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    if (myMnemonic == "tw" &&
        (srcRegIdx(0).index() == 0) && (srcRegIdx(1).index() == 0)) {
        myMnemonic = "trap";
        printSrcs = false;
    } else {
        ext = suffix();
        if (!ext.empty() &&
            (myMnemonic == "tw" || myMnemonic == "td")) {
            myMnemonic += ext;
        } else {
            printCond = true;
        }
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the trap condition
    if (printCond)
        ss << (int) to;

    // Print the source registers
    if (printSrcs) {
        if (_numSrcRegs > 0) {
            if (printCond)
                ss << ", ";
            printReg(ss, srcRegIdx(0));
        }

        if (_numSrcRegs > 1) {
            ss << ", ";
            printReg(ss, srcRegIdx(1));
        }
    }

    return ss.str();
}


std::string
IntImmTrapOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::string ext;
    std::stringstream ss;
    bool printCond = false;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);

    // Special cases
    ext = suffix();
    if (!ext.empty()) {
        if (myMnemonic == "twi") {
            myMnemonic = "tw" + ext + "i";
        } else if (myMnemonic == "tdi") {
            myMnemonic = "td" + ext + "i";
        } else {
            printCond = true;
        }
    } else {
        printCond = true;
    }

    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the trap condition
    if (printCond)
        ss << (int) to;

    // Print the source registers
    if (_numSrcRegs > 0) {
        if (printCond)
            ss << ", ";
        printReg(ss, srcRegIdx(0));
    }

    // Print the immediate value
    ss << ", " << si;

    return ss.str();
}

const uint8_t
Crypto::aesSBOX[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b,
    0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
    0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
    0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
    0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
    0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed,
    0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
    0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec,
    0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14,
    0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
    0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
    0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f,
    0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
    0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
    0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
    0xb0, 0x54, 0xbb, 0x16
};

const uint8_t
Crypto::aesInvSBOX[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e,
    0x81, 0xf3, 0xd7, 0xfb, 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87,
    0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb, 0x54, 0x7b, 0x94, 0x32,
    0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49,
    0x6d, 0x8b, 0xd1, 0x25, 0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16,
    0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92, 0x6c, 0x70, 0x48, 0x50,
    0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05,
    0xb8, 0xb3, 0x45, 0x06, 0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02,
    0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b, 0x3a, 0x91, 0x11, 0x41,
    0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8,
    0x1c, 0x75, 0xdf, 0x6e, 0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89,
    0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b, 0xfc, 0x56, 0x3e, 0x4b,
    0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59,
    0x27, 0x80, 0xec, 0x5f, 0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d,
    0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef, 0xa0, 0xe0, 0x3b, 0x4d,
    0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63,
    0x55, 0x21, 0x0c, 0x7d
};

const uint8_t
Crypto::aesFFLOG[256] = {
    0x00, 0x00, 0x19, 0x01, 0x32, 0x02, 0x1a, 0xc6, 0x4b, 0xc7, 0x1b, 0x68,
    0x33, 0xee, 0xdf, 0x03, 0x64, 0x04, 0xe0, 0x0e, 0x34, 0x8d, 0x81, 0xef,
    0x4c, 0x71, 0x08, 0xc8, 0xf8, 0x69, 0x1c, 0xc1, 0x7d, 0xc2, 0x1d, 0xb5,
    0xf9, 0xb9, 0x27, 0x6a, 0x4d, 0xe4, 0xa6, 0x72, 0x9a, 0xc9, 0x09, 0x78,
    0x65, 0x2f, 0x8a, 0x05, 0x21, 0x0f, 0xe1, 0x24, 0x12, 0xf0, 0x82, 0x45,
    0x35, 0x93, 0xda, 0x8e, 0x96, 0x8f, 0xdb, 0xbd, 0x36, 0xd0, 0xce, 0x94,
    0x13, 0x5c, 0xd2, 0xf1, 0x40, 0x46, 0x83, 0x38, 0x66, 0xdd, 0xfd, 0x30,
    0xbf, 0x06, 0x8b, 0x62, 0xb3, 0x25, 0xe2, 0x98, 0x22, 0x88, 0x91, 0x10,
    0x7e, 0x6e, 0x48, 0xc3, 0xa3, 0xb6, 0x1e, 0x42, 0x3a, 0x6b, 0x28, 0x54,
    0xfa, 0x85, 0x3d, 0xba, 0x2b, 0x79, 0x0a, 0x15, 0x9b, 0x9f, 0x5e, 0xca,
    0x4e, 0xd4, 0xac, 0xe5, 0xf3, 0x73, 0xa7, 0x57, 0xaf, 0x58, 0xa8, 0x50,
    0xf4, 0xea, 0xd6, 0x74, 0x4f, 0xae, 0xe9, 0xd5, 0xe7, 0xe6, 0xad, 0xe8,
    0x2c, 0xd7, 0x75, 0x7a, 0xeb, 0x16, 0x0b, 0xf5, 0x59, 0xcb, 0x5f, 0xb0,
    0x9c, 0xa9, 0x51, 0xa0, 0x7f, 0x0c, 0xf6, 0x6f, 0x17, 0xc4, 0x49, 0xec,
    0xd8, 0x43, 0x1f, 0x2d, 0xa4, 0x76, 0x7b, 0xb7, 0xcc, 0xbb, 0x3e, 0x5a,
    0xfb, 0x60, 0xb1, 0x86, 0x3b, 0x52, 0xa1, 0x6c, 0xaa, 0x55, 0x29, 0x9d,
    0x97, 0xb2, 0x87, 0x90, 0x61, 0xbe, 0xdc, 0xfc, 0xbc, 0x95, 0xcf, 0xcd,
    0x37, 0x3f, 0x5b, 0xd1, 0x53, 0x39, 0x84, 0x3c, 0x41, 0xa2, 0x6d, 0x47,
    0x14, 0x2a, 0x9e, 0x5d, 0x56, 0xf2, 0xd3, 0xab, 0x44, 0x11, 0x92, 0xd9,
    0x23, 0x20, 0x2e, 0x89, 0xb4, 0x7c, 0xb8, 0x26, 0x77, 0x99, 0xe3, 0xa5,
    0x67, 0x4a, 0xed, 0xde, 0xc5, 0x31, 0xfe, 0x18, 0x0d, 0x63, 0x8c, 0x80,
    0xc0, 0xf7, 0x70, 0x07
};

const uint8_t
Crypto::aesFFEXP[256] = {
    0x01, 0x03, 0x05, 0x0f, 0x11, 0x33, 0x55, 0xff, 0x1a, 0x2e, 0x72, 0x96,
    0xa1, 0xf8, 0x13, 0x35, 0x5f, 0xe1, 0x38, 0x48, 0xd8, 0x73, 0x95, 0xa4,
    0xf7, 0x02, 0x06, 0x0a, 0x1e, 0x22, 0x66, 0xaa, 0xe5, 0x34, 0x5c, 0xe4,
    0x37, 0x59, 0xeb, 0x26, 0x6a, 0xbe, 0xd9, 0x70, 0x90, 0xab, 0xe6, 0x31,
    0x53, 0xf5, 0x04, 0x0c, 0x14, 0x3c, 0x44, 0xcc, 0x4f, 0xd1, 0x68, 0xb8,
    0xd3, 0x6e, 0xb2, 0xcd, 0x4c, 0xd4, 0x67, 0xa9, 0xe0, 0x3b, 0x4d, 0xd7,
    0x62, 0xa6, 0xf1, 0x08, 0x18, 0x28, 0x78, 0x88, 0x83, 0x9e, 0xb9, 0xd0,
    0x6b, 0xbd, 0xdc, 0x7f, 0x81, 0x98, 0xb3, 0xce, 0x49, 0xdb, 0x76, 0x9a,
    0xb5, 0xc4, 0x57, 0xf9, 0x10, 0x30, 0x50, 0xf0, 0x0b, 0x1d, 0x27, 0x69,
    0xbb, 0xd6, 0x61, 0xa3, 0xfe, 0x19, 0x2b, 0x7d, 0x87, 0x92, 0xad, 0xec,
    0x2f, 0x71, 0x93, 0xae, 0xe9, 0x20, 0x60, 0xa0, 0xfb, 0x16, 0x3a, 0x4e,
    0xd2, 0x6d, 0xb7, 0xc2, 0x5d, 0xe7, 0x32, 0x56, 0xfa, 0x15, 0x3f, 0x41,
    0xc3, 0x5e, 0xe2, 0x3d, 0x47, 0xc9, 0x40, 0xc0, 0x5b, 0xed, 0x2c, 0x74,
    0x9c, 0xbf, 0xda, 0x75, 0x9f, 0xba, 0xd5, 0x64, 0xac, 0xef, 0x2a, 0x7e,
    0x82, 0x9d, 0xbc, 0xdf, 0x7a, 0x8e, 0x89, 0x80, 0x9b, 0xb6, 0xc1, 0x58,
    0xe8, 0x23, 0x65, 0xaf, 0xea, 0x25, 0x6f, 0xb1, 0xc8, 0x43, 0xc5, 0x54,
    0xfc, 0x1f, 0x21, 0x63, 0xa5, 0xf4, 0x07, 0x09, 0x1b, 0x2d, 0x77, 0x99,
    0xb0, 0xcb, 0x46, 0xca, 0x45, 0xcf, 0x4a, 0xde, 0x79, 0x8b, 0x86, 0x91,
    0xa8, 0xe3, 0x3e, 0x42, 0xc6, 0x51, 0xf3, 0x0e, 0x12, 0x36, 0x5a, 0xee,
    0x29, 0x7b, 0x8d, 0x8c, 0x8f, 0x8a, 0x85, 0x94, 0xa7, 0xf2, 0x0d, 0x17,
    0x39, 0x4b, 0xdd, 0x7c, 0x84, 0x97, 0xa2, 0xfd, 0x1c, 0x24, 0x6c, 0xb4,
    0xc7, 0x52, 0xf6, 0x01
};

const uint8_t
Crypto::aesSHIFT[16] = {
    0, 5, 10, 15, 4, 9, 14, 3,
    8, 13, 2, 7, 12, 1, 6, 11
};

const uint8_t
Crypto::aesINVSHIFT[16] = {
    0, 13, 10, 7, 4, 1, 14, 11,
    8, 5, 2, 15, 12, 9, 6, 3
};

uint8_t
Crypto::aesFFMul(uint8_t a, uint8_t b)
{
    unsigned int log_prod;

    if ((a ==0)|| (b == 0)) return 0;

    log_prod = (aesFFLOG[a] + aesFFLOG[b]);

    if (log_prod > 0xff)
        log_prod = log_prod - 0xff;

    return aesFFEXP[log_prod];
}

void
Crypto::aesSubBytes(uint8_t *output, uint8_t *input)
{
    for (int i = 0; i < 16; ++i) {
        output[i] = aesSBOX[input[i]];
    }
}

void
Crypto::aesInvSubBytes(uint8_t *output, uint8_t *input)
{
    for (int i = 0; i < 16; ++i) {
        output[i] = aesInvSBOX[input[i]];
    }
}

void
Crypto::aesShiftRows(uint8_t *output, uint8_t *input)
{
    for (int i = 0; i < 16; ++i) {
        output[i] = input[aesSHIFT[i]];
    }
}

void
Crypto::aesInvShiftRows(uint8_t *output, uint8_t *input)
{
    for (int i = 0; i < 16; ++i) {
        output[i] = input[aesINVSHIFT[i]];
    }
}

void
Crypto::aesAddRoundKey(uint8_t *output, uint8_t *input,
                    uint8_t *key)
{
    for (int i = 0; i < 16; ++i) {
        output[i] = input[i] ^ key[i];
    }
}

void
Crypto::aesMixColumns(uint8_t *output, uint8_t *input)
{
    for (int j = 0; j < 4; ++j) {
        int row0 = (j * 4);
        int row1 = row0 + 1;
        int row2 = row0 + 2;
        int row3 = row0 + 3;
        uint8_t t1 = input[row0] ^ input[row1] ^
                           input[row2] ^ input[row3];

        output[row1] = input[row1] ^ t1 ^ aesFFMul2(input[row1] ^ input[row2]);
        output[row2] = input[row2] ^ t1 ^ aesFFMul2(input[row2] ^ input[row3]);
        output[row3] = input[row3] ^ t1 ^ aesFFMul2(input[row3] ^ input[row0]);
        output[row0] = input[row0] ^ t1 ^ aesFFMul2(input[row0] ^ input[row1]);
    }
}

void
Crypto::aesInvMixColumns(uint8_t *output, uint8_t *input)
{
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            int index0 = (j * 4) + i;
            int index1 = (j * 4) + ((i + 1) % 4);
            int index2 = (j * 4) + ((i + 2) % 4);
            int index3 = (j * 4) + ((i + 3) % 4);
            output [index0] =
                aesFFMul(0x0e, input[index0]) ^ aesFFMul(0x0b, input[index1]) ^
                aesFFMul(0x0d, input[index2]) ^ aesFFMul(0x09, input[index3]);
            }
    }
}

} // namespace gem5

void bcd_ADD(uint8_t *p_t, uint8_t *p_a, uint8_t *p_b, bool PS)               // wwf
{
  bool sign_a, sign_b;
  sign_a = ((p_a[15] & 0x0f) == 0xB) | ((p_a[15] & 0x0f) == 0xD);
  sign_b = ((p_b[15] & 0x0f) == 0xB) | ((p_b[15] & 0x0f) == 0xD);
  
  // check if a or b are undefined.
  inv_flag = (p_a[15] & 0x0f) < 0xA | (p_b[15] & 0x0f) < 0xA; 
  for(int i=0; i<15; i++)
  {
    inv_flag = inv_flag | ((p_a[i]>>4) > 0x9) | ((p_b[i]>>4) > 0x9);
    inv_flag = inv_flag | ((p_a[i]&0x0F) > 0x9) | ((p_b[i]&0x0F) > 0x9);
  }
  inv_flag = inv_flag | ((p_a[15]>>4) > 0x9) | ((p_b[15]>>4) > 0x9);
  
  if(inv_flag)
  { lt_flag = false; gt_flag = false; eq_flag = false;    return;  }
  
  for(int i=0; i<16; i++)
    p_t[i] = 0x0;
  
  if(sign_a == sign_b)
  {
    uint8_t sum, offset=6, inc=0, t, k;
    for(int i=30; i >= 0; i--)
    {
      t = i % 2;
      k = i / 2;
      if(t == 0)
        sum = (p_a[k] >> 4) + (p_b[k] >> 4) + inc;
      else
        sum = (p_a[k] & 0x0F) + (p_b[k] & 0x0F) + inc;
      if(sum > 9)
      {
        inc = 1;
        sum = (sum + offset) & 0x0F;
      }
      else
        inc = 0;
      if(t == 0)
        p_t[k] = p_t[k] | ((sum << 4) & 0xFF);
      else
        p_t[k] = p_t[k] | (sum & 0x0F);
    }
    if(inc == 1)
      ox_flag = 1;
    else  ox_flag = 0;
    p_t[15] = p_t[15] | (p_a[15] & 0x0f);
  }
  else
  {
    if(!sign_a)   // a >= 0  and b < 0
    {
      uint8_t tmp[18], *p_tmpa;
      for(int i=0; i<16; i++)
        tmp[i] = p_b[i];
      tmp[15] = tmp[15] & 0xF0;
      tmp[15] = tmp[15] | 0x0C;        // -b
      p_tmpa = &tmp[0];
      
      printf("vector      0 - b :                          ");
      for(int i=0; i<16; i++)
        printf("%02x", p_tmpa[i]);
      printf("\n");
      printf("vector      start doing     a - (0 - b)\n");
      bcd_SUBTRACT(p_t, p_a, p_tmpa, PS);
    }
    else          // a < 0  and b >= 0
    {
      uint8_t tmp[18], *p_tmpa;
      for(int i=0; i<16; i++)
        tmp[i] = p_a[i];
      tmp[15] = tmp[15] & 0xF0;
      tmp[15] = tmp[15] | 0x0C;        // -a
      p_tmpa = &tmp[0];
      
      printf("vector      0 - a :                         ");
      for(int i=0; i<16; i++)
        printf("%02x", p_tmpa[i]);
      printf("\n");
      printf("vector      start doing     b - (0 - a)\n");
      bcd_SUBTRACT(p_t, p_b, p_tmpa, PS);
    }
  }
  
  // set control flags.
  bool src_sign = ((p_t[15] & 0x0f) == 0xB) | ((p_t[15] & 0x0f) == 0xD);
  eq_flag = true;
  for(int i=0; i<15; i++)
    eq_flag = eq_flag & (p_t[i] == 0x0);
  eq_flag = eq_flag & ((p_t[15]>>4) == 0x0);
  lt_flag = (!eq_flag) & src_sign;
  gt_flag = (!eq_flag) & (!src_sign);
  
  p_t[15] = p_t[15] & 0xF0;
  if(!lt_flag)
  {
    if(PS == 0)
      p_t[15] = p_t[15] | 0b1100;
    else
      p_t[15] = p_t[15] | 0b1111;
  }
  else
    p_t[15] = p_t[15] | 0b1101;
}

void bcd_SUBTRACT(uint8_t *p_t, uint8_t *p_a, uint8_t *p_b, bool PS)               // wwf  a - b
{
  bool sign_a, sign_b;
  sign_a = ((p_a[15] & 0x0f) == 0xB) | ((p_a[15] & 0x0f) == 0xD);
  sign_b = ((p_b[15] & 0x0f) == 0xB) | ((p_b[15] & 0x0f) == 0xD);
  
  // check if a or b are undefined.
  inv_flag = (p_a[15] & 0x0f) < 0xA | (p_b[15] & 0x0f) < 0xA; 
  for(int i=0; i<15; i++)
  {
    inv_flag = inv_flag | ((p_a[i]>>4) > 0x9) | ((p_b[i]>>4) > 0x9);
    inv_flag = inv_flag | ((p_a[i]&0x0F) > 0x9) | ((p_b[i]&0x0F) > 0x9);
  }
  inv_flag = inv_flag | ((p_a[15]>>4) > 0x9) | ((p_b[15]>>4) > 0x9);
  
  if(inv_flag)
  { lt_flag = false; gt_flag = false; eq_flag = false;    return;  }
  ox_flag = 0;
  
  for(int i=0; i<16; i++)
    p_t[i] = 0x0;
  
  if(sign_a == sign_b)
  {
    int8_t sub, offset=6, inc=0, t, k;
    
    // check if a > b not consider the sign.
    bool check_flag = true;
    uint8_t *p_tmp, tmp_a, tmp_b;    
    for(int i=0; i < 31; i++)
    {
      t = i % 2;
      k = i / 2;
      if(t == 0)
      {
        tmp_a = p_a[k] >> 4;
        tmp_b = p_b[k] >> 4;
      }
      else
      {
        tmp_a = p_a[k] & 0x0F;
        tmp_b = p_b[k] & 0x0F;
      }
      if(tmp_a < tmp_b)               // a < b , not consider the sign
      {
        check_flag = false;
        p_tmp = p_a;   p_a = p_b;    p_b = p_tmp;
        break;
      }
      else if(tmp_a > tmp_b)
        break;
    }
    
    for(int i=30; i >= 0; i--)
    {
      t = i % 2;
      k = i / 2;
      if(t == 0)
      {
        sub = (p_a[k] >> 4) - inc;
        if(sub < 0)
        {
          sub = sub + 16 - (p_b[k] >> 4);
          inc = 1;
        }
        else
        {
          inc = 0;                        // reset inc.
          sub = sub - (p_b[k] >> 4);
          if(sub < 0)
          {
            sub = sub + 16;
            inc = 1;
          }
        }
        
      }
      else
      {
        sub = (p_a[k] & 0x0F) - inc;
        if(sub < 0)
        {
          sub = sub + 16 - (p_b[k] & 0x0F);
          inc = 1;
        }
        else
        {
          inc = 0;                        // reset inc.
          sub = sub - (p_b[k] & 0x0F);
          if(sub < 0)
          {
            sub = sub + 16;
            inc = 1;
          }
        }
        
      }
        
      //printf("vector  1   i=%d k=%d t=%d  a=%x b=%x sub=%x  inc=%d   sign=%d\n", i, k, t, p_a[k], p_b[k], sub, inc, sign_a);
      if(inc == 1)
      {
        sub = (sub - offset) & 0x0F;
      }
      //printf("vector      i=%d k=%d t=%d  a=%x b=%x sub=%x  inc=%d   sign=%d\n", i, k, t, p_a[k], p_b[k], sub, inc, sign_a);
      if(t == 0)
        p_t[k] = p_t[k] | ((sub << 4) & 0xFF);
      else
        p_t[k] = p_t[k] | (sub & 0x0F);
    }
    
    printf("vector    check_flag=%d   inc=%d\n", check_flag, inc);
    if(!check_flag)
    {
      if(sign_a)
        p_t[15] = p_t[15] | 0b1100;
      else
        p_t[15] = p_t[15] | 0b1101;
    }
    else  
      p_t[15] = p_t[15] | (p_a[15] & 0x0f);
  }
  else
  {    
    if(!sign_a)   // a >= 0  and b < 0
    {
      uint8_t tmp[18], *p_tmpa;
      for(int i=0; i<16; i++)
        tmp[i] = p_b[i];
      tmp[15] = tmp[15] & 0xF0;
      tmp[15] = tmp[15] | 0x0C;        // -b
      p_tmpa = &tmp[0];
      
      printf("vector      0 - b :                          ");
      for(int i=0; i<16; i++)
        printf("%02x", p_tmpa[i]);
      printf("\n");
      printf("vector      start doing     a + (0 - b)\n");
      bcd_ADD(p_t, p_a, p_tmpa, PS);
    }
    else          // a < 0  and b >= 0
    {
      uint8_t tmp[18], *p_tmpa;
      for(int i=0; i<16; i++)
        tmp[i] = p_b[i];
      tmp[15] = tmp[15] & 0xF0;
      tmp[15] = tmp[15] | 0x0D;        // -b
      p_tmpa = &tmp[0];
      
      printf("vector      0 - b :                         ");
      for(int i=0; i<16; i++)
        printf("%02x", p_tmpa[i]);
      printf("\n");
      printf("vector      start doing     a + (0 - b)\n");
      bcd_ADD(p_t, p_a, p_tmpa, PS);
    }
  }
  
  // set control flags.
  bool src_sign = ((p_t[15] & 0x0f) == 0xB) | ((p_t[15] & 0x0f) == 0xD);
  eq_flag = true;
  for(int i=0; i<15; i++)
    eq_flag = eq_flag & (p_t[i] == 0x0);
  eq_flag = eq_flag & ((p_t[15]>>4) == 0x0);
  lt_flag = (!eq_flag) & src_sign;
  gt_flag = (!eq_flag) & (!src_sign);
  
  p_t[15] = p_t[15] & 0xF0;
  if(!lt_flag)
  {
    if(PS == 0)
      p_t[15] = p_t[15] | 0b1100;
    else
      p_t[15] = p_t[15] | 0b1111;
  }
  else
    p_t[15] = p_t[15] | 0b1101;
}


void print128(__int128_t x)  
{  
    if(x < 0)
    {
        x = -x;
        putchar('-');
    }
    if(!x)  
    {  
        puts("0");  
        return ;  
    }  
    std::string ret="";  
    while(x)  
    {  
        ret+=x%10+'0';  
        x/=10;  
    }  
    std::reverse(ret.begin(),ret.end());  
    std::cout<<ret<<std::endl;  
}  
void assign_128(__int128 &x, char num[])
{
    x = 0;
    int f = 1, i=0;
    char ch;
    int n = strlen(num);
    if(num[0] == '-') 
    { f = -f;  i = 1;  }
    for(; i < n; i++)
    {
      ch = num[i];
      x = x*10 + ch-'0';
    }
    x *= f;
}

//Let x be a signed integer quadword.
//Let y indicate the preferred sign code.
//Return the signed integer value x in packed decimal format.
void bcd_CONVERT_FROM_SI128(uint8_t *p_t, __int128_t x, bool y)
{
        __int128_t zero = 0x0;
	
	printf("vector:  bcd_CONVERT_FROM_SI128  data = ");
	print128(x);
        uint64_t *pu = (uint64_t *)&x;
        printf("vector:    x=    %llx    %llx\n", *pu, *(pu+1));
	
        uint8_t sign;
        uint8_t *p8 = (uint8_t *)&x;
        
        if( x < zero)
        {
	  x = ~x + 1;
	  printf("vector:  x = %x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", \
		                                 p8[0],p8[1],p8[2],p8[3],p8[4],p8[5],p8[6],p8[7], \
		                                 p8[8],p8[9],p8[10],p8[11],p8[12],p8[13],p8[14],p8[15]);
	  sign = 0x000D;
	}
	else
	  sign = (y==0) ? 0x000C : 0x000F;
	printf("vector:     sign=%d\n", sign);
          
        printf("vector: start     vrb =  ");
        print128(x);
        pu = (uint64_t *)&x;
        printf("vector:        %llx    %llx\n", *pu, *(pu+1));
	
	for(int i=0; i<16; i++)
	  p_t[i] = 0x0;
	int8_t digit, i, j, t;
	for(i=30; i > -1; i--)
	{
	  if(x <= 0)
	    break;
	  t = i % 2;
	  j = i / 2;
	  digit = x % 10;
	  if(t == 0)
	    p_t[j] = p_t[j] | (digit << 4);
	  else
	    p_t[j] = p_t[j] | digit;	
	  printf("vector:  digit= %d   data= ", digit);  
          print128(x);
	  x = x / 10;
	}
	p_t[15] = p_t[15] | (sign & 0x0F);
}

void si128_CONVERT_FROM_BCD(uint8_t *p_t, uint8_t *p8)
{
        __int128_t result = 0x0;
	
	//printf("vector:    si128_CONVERT_FROM_BCD  data =  %x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", \
		                                 p8[0],p8[1],p8[2],p8[3],p8[4],p8[5],p8[6],p8[7], \
		                                 p8[8],p8[9],p8[10],p8[11],p8[12],p8[13],p8[14],p8[15]);
	
        __int128_t scale = 1, digit;
        int sign = p8[15] & 0x0F;
        int t, j;
        for(int i=30; i>-1; i--)
        {
	  t = i % 2;
	  j = i / 2;
	  if(t == 0)
	    digit = (p8[j] >> 4) & 0x0F;
	  else
	    digit = p8[j] & 0x0F;
	  result = result + digit * scale;
	  scale = scale * 10;
	}
	
	//printf("vector: result =  ");
        //print128(result);
        
	if (sign == 0x0B | sign == 0x0D)
	 result = ~result + 1;
	
	//printf("vector: result =  ");
        //print128(result);
	
	uint8_t *ppp = (uint8_t*)&result;
	for(int i = 0; i < 16; i++)
	  p_t[15-i] = ppp[i];
	
	//printf("vector:    return data =  %x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", \
		                                 p_t[0],p_t[1],p_t[2],p_t[3],p_t[4],p_t[5],p_t[6],p_t[7], \
		                                 p_t[8],p_t[9],p_t[10],p_t[11],p_t[12],p_t[13],p_t[14],p_t[15]);
}

