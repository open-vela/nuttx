/****************************************************************************
 * arch/loongarch/include/csr.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_LOONGARCH_INCLUDE_CSR_H
#define __ARCH_LOONGARCH_INCLUDE_CSR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__
#define _ULCAST_
#else
#define _ULCAST_ (unsigned long)
#endif

/* Basic CSR registers */

#define LOONGARCH_CSR_CRMD         0x0
#define  CSR_CRMD_DACM_SHIFT       7
#define  CSR_CRMD_DACM             (_ULCAST_(0x3) << CSR_CRMD_DACM_SHIFT)
#define  CSR_CRMD_DACF_SHIFT       5
#define  CSR_CRMD_DACF             (_ULCAST_(0x3) << CSR_CRMD_DACF_SHIFT)
#define  CSR_CRMD_PG_SHIFT         4
#define  CSR_CRMD_PG               (_ULCAST_(0x1) << CSR_CRMD_PG_SHIFT)
#define  CSR_CRMD_DA_SHIFT         3
#define  CSR_CRMD_DA               (_ULCAST_(0x1) << CSR_CRMD_DA_SHIFT)
#define  CSR_CRMD_IE_SHIFT         2
#define  CSR_CRMD_IE               (_ULCAST_(0x1) << CSR_CRMD_IE_SHIFT)
#define  CSR_CRMD_PLV_SHIFT        0
#define  CSR_CRMD_PLV              (_ULCAST_(0x3) << CSR_CRMD_PLV_SHIFT)

#define PLV_KERN                   0
#define PLV_USER                   3
#define PLV_MASK                   0x3

#define LOONGARCH_CSR_PRMD         0x1
#define  CSR_PRMD_PIE_SHIFT        2
#define  CSR_PRMD_PIE              (_ULCAST_(0x1) << CSR_PRMD_PIE_SHIFT)
#define  CSR_PRMD_PPLV_SHIFT       0
#define  CSR_PRMD_PPLV             (_ULCAST_(0x3) << CSR_PRMD_PPLV_SHIFT)

#define LOONGARCH_CSR_EUEN         0x2
#define  CSR_EUEN_FPEN_SHIFT       0
#define  CSR_EUEN_FPEN             (_ULCAST_(0x1) << CSR_EUEN_FPEN_SHIFT)

#define LOONGARCH_CSR_MISC         0x3

#define LOONGARCH_CSR_ECFG         0x4
#define  CSR_ECFG_VS_SHIFT         16
#define  CSR_ECFG_VS               (_ULCAST_(0x7) << CSR_ECFG_VS_SHIFT)
#define  CSR_ECFG_IM_SHIFT         0
#define  CSR_ECFG_IM               (_ULCAST_(0x1fff) << CSR_ECFG_IM_SHIFT)

#define LOONGARCH_CSR_ESTAT        0x5
#define  CSR_ESTAT_ESUBCODE_SHIFT  22
#define  CSR_ESTAT_ESUBCODE        (_ULCAST_(0x1ff) << CSR_ESTAT_ESUBCODE_SHIFT)
#define  CSR_ESTAT_EXC_SHIFT       16
#define  CSR_ESTAT_EXC_WIDTH       6
#define  CSR_ESTAT_EXC             (_ULCAST_(0x3f) << CSR_ESTAT_EXC_SHIFT)
#define  CSR_ESTAT_IS_SHIFT        0
#define  CSR_ESTAT_IS_WIDTH        15
#define  CSR_ESTAT_IS              (_ULCAST_(0x7fff) << CSR_ESTAT_IS_SHIFT)

#define LOONGARCH_CSR_ERA          0x6
#define LOONGARCH_CSR_BADV         0x7
#define LOONGARCH_CSR_BADI         0x8
#define LOONGARCH_CSR_EBASE        0xc

/* TLB related CSR registers */

#define LOONGARCH_CSR_TLBIDX       0x10
#define LOONGARCH_CSR_TLBEHI       0x11
#define LOONGARCH_CSR_TLBELO0      0x12
#define LOONGARCH_CSR_TLBELO1      0x13
#define LOONGARCH_CSR_ASID         0x18
#define LOONGARCH_CSR_PGDL         0x19
#define LOONGARCH_CSR_PGDH         0x1a
#define LOONGARCH_CSR_PGD          0x1b
#define LOONGARCH_CSR_PWCTL0       0x1c
#define LOONGARCH_CSR_PWCTL1       0x1d
#define LOONGARCH_CSR_STLBPGSIZE   0x1e
#define LOONGARCH_CSR_RVACFG       0x1f

#define LOONGARCH_CSR_TLBRENTRY    0x88
#define LOONGARCH_CSR_TLBRBADV     0x89
#define LOONGARCH_CSR_TLBRERA      0x8a
#define LOONGARCH_CSR_TLBREBASE    0x88

/* Performance monitoring registers */

#define LOONGARCH_CSR_IMPCTL1      0x80
#define LOONGARCH_CSR_PERFCTRL0    0x200

/* Loongson extended config registers */

#define LOONGARCH_CSR_PRID         0xc0
#define LOONGARCH_CSR_MCSR1        0xc1
#define LOONGARCH_CSR_MCSR2        0xc2
#define LOONGARCH_CSR_MCSR9        0xc9
#define LOONGARCH_CSR_MCSR24       0xf0

/* Config CSR registers */

#define LOONGARCH_CSR_CPUNUM       0x20
#define LOONGARCH_CSR_PRCFG1       0x21
#define LOONGARCH_CSR_PRCFG2       0x22
#define LOONGARCH_CSR_PRCFG3       0x23

/* Kscratch registers */

#define LOONGARCH_CSR_KS0          0x30
#define LOONGARCH_CSR_KS1          0x31
#define LOONGARCH_CSR_KS2          0x32
#define LOONGARCH_CSR_KS3          0x33
#define LOONGARCH_CSR_KS4          0x34
#define LOONGARCH_CSR_KS5          0x35
#define LOONGARCH_CSR_KS6          0x36
#define LOONGARCH_CSR_KS7          0x37
#define LOONGARCH_CSR_KS8          0x38

/* Timer registers */

#define LOONGARCH_CSR_TMID         0x40
#define LOONGARCH_CSR_TCFG         0x41
#define  CSR_TCFG_VAL_SHIFT        2
#define  CSR_TCFG_VAL              (_ULCAST_(0x3fffffffffff) << CSR_TCFG_VAL_SHIFT)
#define  CSR_TCFG_PERIOD_SHIFT     1
#define  CSR_TCFG_PERIOD           (_ULCAST_(0x1) << CSR_TCFG_PERIOD_SHIFT)
#define  CSR_TCFG_EN               (_ULCAST_(0x1))

#define LOONGARCH_CSR_TVAL         0x42
#define LOONGARCH_CSR_CNTC         0x43
#define LOONGARCH_CSR_TINTCLR      0x44
#define  CSR_TINTCLR_TI            (_ULCAST_(0x1))

/* Direct map windows registers */

#define LOONGARCH_CSR_DMWIN0       0x180
#define LOONGARCH_CSR_DMWIN1       0x181
#define LOONGARCH_CSR_DMWIN2       0x182
#define LOONGARCH_CSR_DMWIN3       0x183

#define DMW_PABITS                 48
#define TO_PHYS_MASK               ((1ULL << DMW_PABITS) - 1)

#define CSR_DMW0_PLV0              (_ULCAST_(1) << 0)
#define CSR_DMW0_VSEG              (_ULCAST_(0x8000))
#define CSR_DMW0_BASE              (CSR_DMW0_VSEG << DMW_PABITS)
#define CSR_DMW0_INIT              (CSR_DMW0_BASE | CSR_DMW0_PLV0)

#define CSR_DMW1_PLV0              (_ULCAST_(1) << 0)
#define CSR_DMW1_MAT               (_ULCAST_(1) << 4)
#define CSR_DMW1_VSEG              (_ULCAST_(0x9000))
#define CSR_DMW1_BASE              (CSR_DMW1_VSEG << DMW_PABITS)
#define CSR_DMW1_INIT              (CSR_DMW1_BASE | CSR_DMW1_MAT | CSR_DMW1_PLV0)

/* Exception codes */

#define EXCCODE_RSV                0
#define EXCCODE_TLBL               1
#define EXCCODE_TLBS               2
#define EXCCODE_TLBI               3
#define EXCCODE_TLBM               4
#define EXCCODE_TLBRI              5
#define EXCCODE_TLBXI              6
#define EXCCODE_TLBPE              7
#define EXCCODE_ADE                8
#define EXCCODE_ALE                9
#define EXCCODE_OOB                10
#define EXCCODE_SYS                11
#define EXCCODE_BP                 12
#define EXCCODE_INE                13
#define EXCCODE_IPE                14
#define EXCCODE_FPDIS              15
#define EXCCODE_LSXDIS             16
#define EXCCODE_LASXDIS            17
#define EXCCODE_FPE                18

#define EXCCODE_INT_START          64
#define EXCCODE_SIP0               64
#define EXCCODE_SIP1               65
#define EXCCODE_IP0                66
#define EXCCODE_IP1                67
#define EXCCODE_IP2                68
#define EXCCODE_IP3                69
#define EXCCODE_IP4                70
#define EXCCODE_IP5                71
#define EXCCODE_IP6                72
#define EXCCODE_IP7                73
#define EXCCODE_PC                 74
#define EXCCODE_TIMER              75
#define EXCCODE_IPI                76
#define EXCCODE_NMI                77
#define EXCCODE_INT_END            78

/* ECFG interrupt bits */

#define ECFGB_SIP0                 0
#define ECFGB_SIP1                 1
#define ECFGB_IP0                  2
#define ECFGB_IP1                  3
#define ECFGB_IP2                  4
#define ECFGB_IP3                  5
#define ECFGB_IP4                  6
#define ECFGB_IP5                  7
#define ECFGB_IP6                  8
#define ECFGB_IP7                  9
#define ECFGB_PC                   10
#define ECFGB_TIMER                11
#define ECFGB_IPI                  12
#define ECFGF(hwirq)               (_ULCAST_(1) << hwirq)

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

static inline unsigned long csr_readq(unsigned int reg)
{
  unsigned long val;

  __asm__ __volatile__ (
    "csrrd %0, %1\n\t"
    : "=r"(val)
    : "i"(reg)
  );

  return val;
}

static inline void csr_writeq(unsigned long val, unsigned int reg)
{
  __asm__ __volatile__ (
    "csrwr %0, %1\n\t"
    :
    : "r"(val), "i"(reg)
    : "memory"
  );
}

static inline unsigned long csr_xchgq(unsigned long val,
                                       unsigned long mask,
                                       unsigned int reg)
{
  unsigned long ret = val;

  __asm__ __volatile__ (
    "csrxchg %0, %1, %2\n\t"
    : "+r"(ret)
    : "r"(mask), "i"(reg)
    : "memory"
  );

  return ret;
}

#define read_csr_crmd()        csr_readq(LOONGARCH_CSR_CRMD)
#define write_csr_crmd(val)    csr_writeq(val, LOONGARCH_CSR_CRMD)
#define read_csr_prmd()        csr_readq(LOONGARCH_CSR_PRMD)
#define write_csr_prmd(val)    csr_writeq(val, LOONGARCH_CSR_PRMD)
#define read_csr_euen()        csr_readq(LOONGARCH_CSR_EUEN)
#define write_csr_euen(val)    csr_writeq(val, LOONGARCH_CSR_EUEN)
#define read_csr_ecfg()        csr_readq(LOONGARCH_CSR_ECFG)
#define write_csr_ecfg(val)    csr_writeq(val, LOONGARCH_CSR_ECFG)
#define read_csr_estat()       csr_readq(LOONGARCH_CSR_ESTAT)
#define write_csr_estat(val)   csr_writeq(val, LOONGARCH_CSR_ESTAT)
#define read_csr_era()         csr_readq(LOONGARCH_CSR_ERA)
#define write_csr_era(val)     csr_writeq(val, LOONGARCH_CSR_ERA)
#define read_csr_ebase()       csr_readq(LOONGARCH_CSR_EBASE)
#define write_csr_ebase(val)   csr_writeq(val, LOONGARCH_CSR_EBASE)
#define read_csr_tcfg()        csr_readq(LOONGARCH_CSR_TCFG)
#define write_csr_tcfg(val)    csr_writeq(val, LOONGARCH_CSR_TCFG)
#define read_csr_tval()        csr_readq(LOONGARCH_CSR_TVAL)
#define write_csr_tval(val)    csr_writeq(val, LOONGARCH_CSR_TVAL)
#define write_csr_tintclr(val) csr_writeq(val, LOONGARCH_CSR_TINTCLR)
#define read_csr_asid()        csr_readq(LOONGARCH_CSR_ASID)
#define write_csr_asid(val)    csr_writeq(val, LOONGARCH_CSR_ASID)

static inline unsigned int read_csr_excode(void)
{
  return (csr_readq(LOONGARCH_CSR_ESTAT) & CSR_ESTAT_EXC) >>
         CSR_ESTAT_EXC_SHIFT;
}

static inline uint64_t drdtime(void)
{
  int rid = 0;
  uint64_t val = 0;

  __asm__ __volatile__ (
    "rdtime.d %0, %1\n\t"
    : "=r"(val), "=r"(rid)
    :
  );

  return val;
}

static inline uint32_t iocsr_read32(uint32_t reg)
{
  uint32_t val;

  __asm__ __volatile__ (
    "iocsrrd.w %0, %1\n\t"
    : "=r"(val)
    : "r"(reg)
  );

  return val;
}

static inline uint32_t read_cpucfg(uint32_t reg)
{
  uint32_t val;

  __asm__ __volatile__ (
    "cpucfg %0, %1\n\t"
    : "=r"(val)
    : "r"(reg)
  );

  return val;
}

static inline void iocsr_write32(uint32_t val, uint32_t reg)
{
  __asm__ __volatile__ (
    "iocsrwr.w %0, %1\n\t"
    :
    : "r"(val), "r"(reg)
    : "memory"
  );
}

static inline uint64_t iocsr_read64(uint32_t reg)
{
  uint64_t val;

  __asm__ __volatile__ (
    "iocsrrd.d %0, %1\n\t"
    : "=r"(val)
    : "r"(reg)
  );

  return val;
}

static inline void iocsr_write64(uint64_t val, uint32_t reg)
{
  __asm__ __volatile__ (
    "iocsrwr.d %0, %1\n\t"
    :
    : "r"(val), "r"(reg)
    : "memory"
  );
}

#define EIOINTC_REG_NODEMAP     0x14a0
#define EIOINTC_REG_IPMAP       0x14c0
#define EIOINTC_REG_ENABLE      0x1600
#define EIOINTC_REG_BOUNCE      0x1680
#define EIOINTC_REG_ISR         0x1800
#define EIOINTC_REG_ROUTE       0x1c00

#define LOONGARCH_IOCSR_MISC_FUNC  0x100
#define IOCSR_MISC_FUNC_EXT_IOI_EN (1ULL << 27)

#define LOONGARCH_CPUCFG4       0x4
#define LOONGARCH_CPUCFG5       0x5

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_LOONGARCH_INCLUDE_CSR_H */
