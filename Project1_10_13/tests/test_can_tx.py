"""Compile the production CAN transmit functions against a mailbox mock (Windows/MSVC)."""
from pathlib import Path
import os
import subprocess
import tempfile


MOCK = r"""
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t ESR; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; } CAN_HandleTypeDef;
typedef struct {
    uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
#define CAN_ESR_BOFF 4U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define DISABLE 0U
static CAN_TypeDef device;
static CAN_HandleTypeDef hcan1 = { &device };
static uint8_t canInitialized, canBusOffActive;
static uint32_t canTxOkCount, canTxErrCount, canTxMailboxWaitCount;
static uint32_t canTxMailboxTimeoutCount, canBusOffCount, canEsr, canFreeMailboxes;
static uint32_t tick, freeSlots, checks, releaseAtCheck, submissions;
static HAL_StatusTypeDef addResult;
static CAN_TxHeaderTypeDef lastHeader;
static uint8_t lastData[8];
static uint32_t HAL_GetTick(void) { return tick++; }
static uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h) {
    (void)h;
    checks++;
    if (!freeSlots && checks >= releaseAtCheck) freeSlots = 1;
    return freeSlots;
}
static HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox) {
    (void)h;
    submissions++;
    if (addResult != HAL_OK) return addResult;
    if (!freeSlots) return HAL_ERROR;
    freeSlots--;
    lastHeader = *header;
    memcpy(lastData, data, 8);
    *mailbox = 1;
    return HAL_OK;
}
HAL_StatusTypeDef CAN_SendFrameWait(uint16_t, const uint8_t *, uint8_t, uint32_t);
static void reset(void) {
    device.ESR = 0;
    canInitialized = 1;
    canBusOffActive = 0;
    canTxOkCount = canTxErrCount = canTxMailboxWaitCount = 0;
    canTxMailboxTimeoutCount = canBusOffCount = canEsr = 0;
    tick = checks = submissions = 0;
    freeSlots = 3;
    releaseAtCheck = UINT32_MAX;
    addResult = HAL_OK;
}
"""

TESTS = r"""
int main(void) {
    uint8_t data[8] = {1,2,3,4,5,6,7,8};
    unsigned i;
    reset();
    assert(CAN_SendFrameWait(0x321, data, 4, 2) == HAL_OK);
    assert(lastHeader.StdId == 0x321 && lastHeader.DLC == 4);
    assert(lastHeader.IDE == CAN_ID_STD && lastHeader.RTR == CAN_RTR_DATA);
    assert(memcmp(lastData, data, 4) == 0);
    for (i=4; i<8; i++) assert(lastData[i] == 0);
    assert(canTxOkCount == 1 && canTxMailboxWaitCount == 0);

    reset();
    canInitialized = 0;
    assert(CAN_SendFrameWait(0x321, data, 8, 2) == HAL_ERROR);
    canInitialized = 1;
    assert(CAN_SendFrameWait(0x800, data, 8, 2) == HAL_ERROR);
    assert(CAN_SendFrameWait(0x321, 0, 8, 2) == HAL_ERROR);
    assert(CAN_SendFrameWait(0x321, data, 0, 2) == HAL_ERROR);
    assert(CAN_SendFrameWait(0x321, data, 9, 2) == HAL_ERROR);
    assert(canTxErrCount == 5 && submissions == 0);

    /* Reproduce the old four-frames/three-mailboxes loss. */
    reset();
    for (i=0; i<3; i++) assert(CAN_SendFrame((uint16_t)(0x321+i), data, 8) == HAL_OK);
    assert(CAN_SendFrame(0x325, data, 8) == HAL_ERROR);
    assert(canTxOkCount == 3 && canTxErrCount == 1);
    assert(canTxMailboxWaitCount == 0);

    reset();
    for (i=0; i<3; i++) assert(CAN_SendFrameWait((uint16_t)(0x321+i), data, 8, 2) == HAL_OK);
    releaseAtCheck = checks + 3;
    assert(CAN_SendFrameWait(0x325, data, 8, 2) == HAL_OK);
    assert(canTxOkCount == 4 && canTxErrCount == 0);
    assert(canTxMailboxWaitCount == 1 && canTxMailboxTimeoutCount == 0);

    reset();
    freeSlots = 0;
    assert(CAN_SendFrameWait(0x325, data, 8, 2) == HAL_TIMEOUT);
    assert(submissions == 0 && canTxErrCount == 1 && canFreeMailboxes == 0);
    assert(canTxMailboxWaitCount == 1 && canTxMailboxTimeoutCount == 1);
    assert(tick == 3);

    reset();
    freeSlots = 0;
    tick = UINT32_MAX - 1;
    assert(CAN_SendFrameWait(0x325, data, 8, 2) == HAL_TIMEOUT);
    assert(tick == 1 && canTxMailboxTimeoutCount == 1);

    reset();
    addResult = HAL_ERROR;
    assert(CAN_SendFrameWait(0x321, data, 8, 2) == HAL_ERROR);
    assert(canTxErrCount == 1 && canTxOkCount == 0);

    reset();
    device.ESR = CAN_ESR_BOFF;
    assert(CAN_SendFrameWait(0x321, data, 8, 2) == HAL_OK);
    assert(CAN_SendFrameWait(0x321, data, 8, 2) == HAL_OK);
    assert(canBusOffCount == 1 && canEsr == CAN_ESR_BOFF);
    device.ESR = 0;
    assert(CAN_SendFrameWait(0x321, data, 8, 2) == HAL_OK);
    assert(canBusOffActive == 0);
    puts("PASS: 8 CAN transmit scenarios (mocked mailbox submission, not hardware ACK).");
    return 0;
}
"""


def main():
    source = (Path(__file__).resolve().parents[1] / "can.c").read_text(encoding="utf-8-sig")
    start = source.index("HAL_StatusTypeDef CAN_SendFrame(")
    end = source.index("HAL_StatusTypeDef CAN_SendByte(")
    vswhere = Path(os.environ["ProgramFiles(x86)"]) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    installation = subprocess.check_output(
        [str(vswhere), "-latest", "-products", "*", "-requires",
         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
        text=True,
    ).strip()
    if not installation:
        raise RuntimeError("MSVC C++ build tools were not found")
    setup = Path(installation) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    with tempfile.TemporaryDirectory(prefix="gyro-can-tx-test-") as temp:
        work = Path(temp)
        (work / "test.c").write_text(MOCK + source[start:end] + TESTS, encoding="utf-8")
        command = f'call "{setup}" >nul && cl /nologo /W4 /WX /utf-8 /TC test.c /Fe:test.exe && test.exe'
        (work / "run.cmd").write_text("@echo off\n" + command + "\n", encoding="utf-8")
        subprocess.run([os.environ["ComSpec"], "/d", "/c", "run.cmd"], cwd=work, check=True)


if __name__ == "__main__":
    main()
