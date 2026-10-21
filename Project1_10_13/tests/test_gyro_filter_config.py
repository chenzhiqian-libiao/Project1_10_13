"""Verify production sensor initialization with a mock register bank, not real SPI."""
from pathlib import Path
import re
import subprocess
import tempfile
from gyro_replay import ROOT, compile_native

MOCK = r"""
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
volatile uint8_t xv7001bbFilterConfigOk, xv7001bbDspCtl1Readback, xv7001bbDspCtl2Readback;
static uint8_t registers[32];
static unsigned calls, failAt, resetAt, lastWriteAt, corruptAddress, corruptAfterReset, waited;
static HAL_StatusTypeDef step(void) {
    return ++calls == failAt ? HAL_TIMEOUT : HAL_OK;
}
static HAL_StatusTypeDef XV7001bb_ReadReg(uint8_t address, uint8_t *value) {
    HAL_StatusTypeDef status = step();
    if (status == HAL_OK) {
        *value = registers[address];
        if (corruptAfterReset && resetAt && address == corruptAddress) *value ^= 2;
    }
    return status;
}
static HAL_StatusTypeDef XV7001bb_WriteData(uint8_t address, uint8_t value) {
    HAL_StatusTypeDef status = step();
    if (status == HAL_OK) {
        registers[address] = value;
        lastWriteAt = calls;
    }
    return status;
}
static HAL_StatusTypeDef XV7001bb_ResetDsp(void) {
    HAL_StatusTypeDef status = step();
    assert(lastWriteAt && calls > lastWriteAt);
    if (status == HAL_OK) resetAt = calls;
    return status;
}
static void HAL_Delay(uint32_t ms) { waited += ms; }
static HAL_StatusTypeDef XV7001bb_SoftReset(void) { return step(); }
static HAL_StatusTypeDef XV7001bb_SleepOut(void) { return step(); }
static void reset(void) {
    unsigned i;
    for(i=0; i<32; i++) registers[i] = 0;
    registers[1] = 0x23;
    registers[2] = 0x46;
    calls = failAt = resetAt = lastWriteAt = corruptAddress = corruptAfterReset = waited = 0;
    xv7001bbFilterConfigOk = 1;
}
"""
TESTS = r"""
int main(void) {
    unsigned i;
    uint8_t expected;
#if XV7001BB_ANTIALIAS_ENABLE
    expected = 0x60;
#else
    expected = 0x46;
#endif
    reset();
    assert(XV7001bb_ConfigureMeasurementFilter() == HAL_OK);
    assert(registers[1] == 0x21 && registers[2] == expected);
    assert(xv7001bbDspCtl1Readback == 0x21 && xv7001bbDspCtl2Readback == expected);
    assert(resetAt == 5 && xv7001bbFilterConfigOk == 1 && calls == 7);
    for(i=1; i<=7; i++) {
        reset();
        failAt = i;
        assert(XV7001bb_ConfigureMeasurementFilter() == HAL_TIMEOUT);
        assert(xv7001bbFilterConfigOk == 0 && calls == i);
    }
    for(i=1; i<=2; i++) {
        reset();
        corruptAfterReset = 1;
        corruptAddress = i;
        assert(XV7001bb_ConfigureMeasurementFilter() == HAL_ERROR);
        assert(xv7001bbFilterConfigOk == 0);
    }
    reset();
    assert(XV7001bb_Init() == HAL_OK);
    assert(xv7001bbFilterConfigOk == 1);
    assert(registers[XV7001BB_REG_OUTCTL1] == XV7001BB_OUTCTL1_DEFAULT);
    assert(registers[XV7001BB_REG_IFCTL] == XV7001BB_IFCTL_SPI_4WIRE);
    assert(waited == XV7001BB_T_IF_MS + XV7001BB_T_STA_MS);
    for(i=1; i<=11; i++) {
        reset();
        failAt = i;
        assert(XV7001bb_Init() == HAL_TIMEOUT);
        assert(xv7001bbFilterConfigOk == 0 && calls == i);
    }
    printf("PASS: 22 register/init scenarios, antialias=%u (mock, not hardware).\n",
           (unsigned)XV7001BB_ANTIALIAS_ENABLE);
    return 0;
}
"""


def function(source, signature):
    start=source.index(signature)
    opening=source.index("{",start)
    depth=1
    end=opening+1
    while depth:
        depth+=(source[end]=="{")-(source[end]=="}")
        end+=1
    return source[start:end]+"\n"


def main():
    source=(ROOT/"xv7001bb.c").read_text(encoding="utf-8-sig")
    header=(ROOT/"xv7001bb.h").read_text(encoding="utf-8-sig")
    names=("XV7001BB_REG_DSPCTL1","XV7001BB_REG_DSPCTL2","XV7001BB_DSPCTL1_ENABLE_HPF",
           "XV7001BB_DSPCTL2_LPF_MASK","XV7001BB_DSPCTL2_LPF_4TH_10HZ",
           "XV7001BB_DSPCTL2_LPF_2ND_100HZ","XV7001BB_T_IF_MS","XV7001BB_T_STA_MS",
           "XV7001BB_REG_IFCTL","XV7001BB_IFCTL_SPI_4WIRE","XV7001BB_REG_OUTCTL1",
           "XV7001BB_OUTCTL1_DEFAULT")
    constants="\n".join(re.search(rf"^#define {name}\s+(\S+)",header,re.M).group()
                        for name in names)+"\n"
    code=MOCK+constants+function(source,"static HAL_StatusTypeDef XV7001bb_ConfigureMeasurementFilter(")
    code+=function(source,"HAL_StatusTypeDef XV7001bb_Init(")+TESTS
    with tempfile.TemporaryDirectory(prefix="gyro-filter-config-") as directory:
        work=Path(directory)
        for enabled in (0,1):
            exe=compile_native(work,f"sensor_{enabled}",code,[f"XV7001BB_ANTIALIAS_ENABLE={enabled}"])
            subprocess.run([str(exe)],check=True)


if __name__=="__main__":
    main()
