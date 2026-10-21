"""Native execution of the production gyro task with a mock clock and SPI input."""
import argparse
import csv
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
PREAMBLE = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
typedef uint32_t TickType_t;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t nowMs;
static int inputCode, inputFailed, haveSample, seeded;
static float seedBias, seedAngle;
static jmp_buf finish;
static uint32_t HAL_GetTick(void) { return nowMs; }
static TickType_t xTaskGetTickCount(void) { return nowMs; }
static void vTaskDelayUntil(TickType_t *, TickType_t);
static HAL_StatusTypeDef XV7001bb_ReadAngle(int16_t *raw) {
    *raw = (int16_t)inputCode;
    return inputFailed ? HAL_ERROR : HAL_OK;
}
static HAL_StatusTypeDef XV7001bb_ReadTmpCelsius(float *temp) {
    *temp = 31.0f;
    return HAL_OK;
}
"""
DRIVER = r"""
static void emit(void) {
    printf("%u,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%u\n",
        nowMs, gyroAccumulatedAngleDeg, gyroRateFilteredDps, gyroBiasDps,
        gyroCalibrationState, gyroZuptState, gyroDataValid, gyroAngleIntegrityLost,
        gyroIntegrationGapCount, gyroRejectedCalibrationCount, gyroCalibrationDegraded,
        gyroCalibrationValidationSamples, gyroCalibrationValidationResidualDps,
        gyroCalibrationValidationPeakDps, gyroCalibrationValidationPassed);
}
static void vTaskDelayUntil(TickType_t *wake, TickType_t period) {
    unsigned tick, command;
    int count;
    (void)wake;
    (void)period;
    if (haveSample) emit();
    count = scanf_s("%u %d %u %d", &tick, &inputCode, &command, &inputFailed);
    if (count == EOF) longjmp(finish, 1);
    if (count != 4 || inputCode < -32768 || inputCode > 32767) exit(2);
    nowMs = tick;
    gyroRequestZeroAngle = (command & 2) != 0;
    gyroRequestRecalibration = (command & 1) != 0;
    haveSample = 1;
}
int main(int argc, char **argv) {
    if (argc == 3) {
        seeded = 1;
        seedBias = strtof(argv[1], NULL);
        seedAngle = strtof(argv[2], NULL);
    } else if (argc != 1) {
        return 2;
    }
    if (!setjmp(finish)) task_gyro(NULL);
    return 0;
}
"""
SEED = r"""
    if (seeded) {
        calibrationState = 2U;
        biasDps = seedBias;
        accumulatedAngleDeg = seedAngle;
        zuptCounter = GYRO_ZUPT_CONFIRM_SAMPLES;
        zuptActive = 1U;
        biasTrackBaseDps = seedBias;
    }
"""


def build(work, median, freeze=None):
    if freeze is None:
        freeze = median
    source = (ROOT / "Project1_10_13.c").read_text(encoding="utf-8-sig")
    declarations = source[source.index("#define CAN_TX_PERIOD_MS"):
                          source.index("static void SystemClock_Config(void);")]
    start = source.index("static float GyroMedianOfThree(")
    end = source.index("\n#endif\n\n#if (MODULE_TASK6_CAN_ENABLE", start)
    task = source[start:end]
    anchor = "\t(void)argument;\n\n\tfor (;;)"
    if task.count(anchor) != 1:
        raise RuntimeError("Gyro task initialization anchor changed")
    task = task.replace(anchor, "\t(void)argument;\n" + SEED + "\n\tfor (;;)")
    header = (ROOT / "xv7001bb.h").read_text(encoding="utf-8-sig")
    sensor = "\n".join(re.search(
        rf"^#define {name}\s+[^\n]+", header, re.M).group()
        for name in ("XV7001BB_SCALE_16BIT", "XV7001BB_RANGE_DPS", "XV7001BB_VALID_RATE_DPS"))
    conversion = "\nstatic float XV7001bb_AngleToDps(int16_t raw) { return (float)raw / XV7001BB_SCALE_16BIT; }\n"
    stem = f"gyro_{median}_{freeze}"
    return compile_native(work, stem, PREAMBLE + sensor + conversion + declarations + task + DRIVER,
                          [f"GYRO_INTEGRATION_MEDIAN3_ENABLE={median}",
                           f"GYRO_ZUPT_FREEZE_ENABLE={freeze}"])


def compile_native(work, stem, source, defines=()):
    (work / f"{stem}.c").write_text(source, encoding="utf-8")
    vswhere = Path(os.environ["ProgramFiles(x86)"]) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    installation = subprocess.check_output(
        [str(vswhere), "-latest", "-products", "*", "-requires",
         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
        text=True).strip()
    if not installation:
        raise RuntimeError("MSVC build tools not found")
    setup = Path(installation) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    # The task has existing debug-only locals unused in the production configuration.
    flags=" ".join("/D"+value for value in defines)
    command = (f'@echo off\ncall "{setup}" >nul && cl /nologo /W3 /WX /wd4101 /wd4189 '
               f'/utf-8 /fp:precise /TC {flags} '
               f'{stem}.c /Fe:{stem}.exe\n')
    (work / "build.cmd").write_text(command, encoding="utf-8")
    subprocess.run([os.environ["ComSpec"], "/d", "/c", "build.cmd"], cwd=work, check=True)
    return work / f"{stem}.exe"


def run(exe, samples, seed=None):
    args = [str(exe)] + ([] if seed is None else [str(seed[0]), str(seed[1])])
    text = "".join(f"{tick & 0xffffffff} {code} {command} {failed}\n"
                   for tick, code, command, failed in samples)
    process = subprocess.run(args, input=text, capture_output=True, text=True, check=True)
    return [tuple(map(float, row.split(","))) for row in process.stdout.splitlines()]


def trajectory(degrees, cycles=1, bias=0):
    speed, acceleration, dt = 100*360/(math.pi*100), 500*360/(math.pi*100), .01
    ramp = speed/acceleration
    total = degrees/speed + ramp
    values = [(0.,0)]*300
    endpoints = []
    for _ in range(cycles):
        for sign in (1,-1):
            for i in range(1, math.ceil(total/dt)+1):
                t = i*dt
                rate = max(0., min(acceleration*t, speed, acceleration*(total-t)))
                values.append((sign*rate,0))
            values.extend([(0.,0)]*300)
        endpoints.append(len(values)-1)
    samples = [((i+1)*10, round((rate+bias)*70), command, 0)
               for i,(rate,command) in enumerate(values)]
    return samples, endpoints


class GyroTests(unittest.TestCase):
    executables = {}

    def test_four_angles_ten_cycles_without_zero_or_recalibration(self):
        for median,exe in self.executables.items():
            for target in (90,180,360,720):
                with self.subTest(median=median, target=target):
                    samples,ends = trajectory(target,10)
                    output = run(exe,samples,seed=(0,0))
                    for end in ends:
                        self.assertLess(abs(output[end][1]),.02)
                        self.assertEqual(output[end][4:8],(2,1,1,0))

    def test_linear_filter_preserves_area_for_asymmetric_oscillations(self):
        samples=[((i+1)*10,0,0,0) for i in range(300)]
        pattern=[100.,100.,130.]
        rates=pattern*100
        samples += [((len(samples)+i+1)*10,round(rate*70),0,0) for i,rate in enumerate(rates)]
        tick=samples[-1][0]
        samples += [(tick+(i+1)*10,0,0,0) for i in range(300)]
        linear=run(self.executables[0],samples,seed=(0,0))[-1][1]
        legacy=run(self.executables[1],samples,seed=(0,0))[-1][1]
        self.assertAlmostEqual(sum(rates)*.01,linear,delta=.02)
        self.assertGreater(abs(legacy-linear),20)

    def test_real_residual_is_not_forced_to_zero(self):
        samples,ends=trajectory(90,10,bias=.2)
        output=run(self.executables[0],samples,seed=(0,0))
        self.assertGreater(output[ends[-1]][1],.5)
        self.assertTrue(all(row[3]==0 for row in output))

    def test_continuous_mode_measures_slow_rotation_without_start_backfill(self):
        rates=[0.]*300+[10.]*100+[100.]*10+[0.]*300
        samples=[((i+1)*10,round(rate*70),0,0) for i,rate in enumerate(rates)]
        result=run(self.executables[0],samples,seed=(0,0))
        self.assertEqual(result[399][5],1)
        self.assertGreater(result[399][1],9)
        self.assertAlmostEqual(result[-1][1],20,delta=.002)

    def test_recalibration_preserves_angle_and_motion_request_is_rejected(self):
        samples,_=trajectory(90)
        samples=samples[:600]
        tick,code,_,failed=samples[330]
        samples[330]=(tick,code,1,failed)
        result=run(self.executables[0],samples,seed=(0,0))
        self.assertEqual(result[-1][9],1)
        held=result[-1][1]
        tick=samples[-1][0]
        samples += [(tick+(i+1)*10,0,1 if i==0 else 0,0) for i in range(8100)]
        output=run(self.executables[0],samples,seed=(0,0))
        self.assertAlmostEqual(held,output[-1][1],delta=.00001)
        self.assertEqual(output[-1][4],2)

    def test_gap_invalidity_survives_recalibration_until_explicit_zero(self):
        samples=[((i+1)*10,0,0,0) for i in range(100)]
        samples += [(1060+i*10,0,1 if i==100 else 0,0) for i in range(8200)]
        output=run(self.executables[0],samples,seed=(0,12.5))
        self.assertEqual(output[-1][6:9],(0,1,1))
        self.assertEqual(output[-1][1],12.5)
        samples.append((samples[-1][0]+10,0,2,0))
        final=run(self.executables[0],samples,seed=(0,12.5))[-1]
        self.assertEqual(final[1],0)
        self.assertEqual(final[6:9],(1,0,1))

    def test_fifty_ms_gap_is_bridged(self):
        samples=[(10,0,0,0),(20,0,0,0),(70,0,0,0)]
        result=run(self.executables[0],samples,seed=(0,0))
        self.assertEqual(result[-1][6:9],(1,0,0))

    def test_spi_failure_and_invalid_range_are_reported(self):
        result=run(self.executables[0],
                   [(10,0,0,0),(20,0,0,1),(30,32767,0,0),(40,0,0,0)],seed=(0,0))
        self.assertEqual([r[6] for r in result],[1,0,0,1])
        self.assertEqual(result[-1][8],0)

    def test_sixty_second_estimate_and_twenty_second_validation(self):
        samples=[((i+1)*10,14,0,0) for i in range(8200)]
        result=run(self.executables[0],samples)
        first=next(i for i,r in enumerate(result) if r[4]==2)
        self.assertEqual(result[first][0],80990)
        self.assertAlmostEqual(result[-1][3],.2,delta=.00001)
        self.assertAlmostEqual(result[-1][1],0,delta=.00001)
        self.assertEqual(result[-1][10:12],(0,2000))
        self.assertEqual(result[-1][14],1)

    @staticmethod
    def calibration_samples(estimate, validation):
        codes=estimate+validation
        return [((i+1)*10,code,1 if i==0 else 0,0) for i,code in enumerate(codes)]

    def test_validation_does_not_refit_bias_or_rewrite_angle(self):
        samples=self.calibration_samples([7]*6000,[8]*2000)
        result=run(self.executables[0],samples,seed=(0,12.5))
        self.assertEqual(result[5999][4],1)
        self.assertEqual(result[7998][3],0)
        self.assertAlmostEqual(result[-1][3],.1,delta=.000001)
        self.assertAlmostEqual(result[-1][12],1/70,delta=.000001)
        self.assertTrue(all(r[1]==12.5 for r in result))
        self.assertEqual(result[-1][10],1)
        self.assertEqual(result[-1][14],0)
        samples += [(80010+i*10,8,0,0) for i in range(200)]
        self.assertGreater(run(self.executables[0],samples,seed=(0,12.5))[-1][1],12.52)

    def test_validation_residual_threshold_on_both_sides(self):
        for sign in (-1,1):
            for count,passed in ((52,1),(53,0)):
                with self.subTest(sign=sign,count=count):
                    validation=([sign]*count+[0]*(500-count))*4
                    result=run(self.executables[0],
                               self.calibration_samples([0]*6000,validation),seed=(0,0))[-1]
                    self.assertAlmostEqual(result[13],count/(70*500),delta=.00000001)
                    self.assertEqual(result[14],passed)
                    self.assertEqual(result[10],1-passed)

    def test_opposite_validation_blocks_cannot_cancel(self):
        validation=[1]*100+[0]*400+[-1]*100+[0]*1400
        result=run(self.executables[0],
                   self.calibration_samples([0]*6000,validation),seed=(0,0))[-1]
        self.assertAlmostEqual(result[12],0,delta=.00000001)
        self.assertGreater(result[13],.0028)
        self.assertEqual(result[10],1)

    def test_validation_checks_raw_not_median_filtered_signal(self):
        validation=([0,0,1]+[0]*47)*40
        result=run(self.executables[0],
                   self.calibration_samples([0]*6000,validation),seed=(0,0))[-1]
        self.assertAlmostEqual(result[12],1/(70*50),delta=.00000001)

    def test_calibration_sampling_faults_degrade_and_reset_on_new_request(self):
        for fault in ("spi","range","gap"):
            with self.subTest(fault=fault):
                samples=self.calibration_samples([0]*6000,[0]*2001)
                if fault=="gap":
                    samples=[(tick+(60 if i>=6100 else 0),code,cmd,failed)
                             for i,(tick,code,cmd,failed) in enumerate(samples)]
                else:
                    tick,_,cmd,_=samples[6100]
                    samples[6100]=(tick,32767 if fault=="range" else 0,cmd,int(fault=="spi"))
                result=run(self.executables[0],samples,seed=(0,3.5))[-1]
                self.assertEqual(result[4],2)
                self.assertEqual(result[10],1)
                tick=samples[-1][0]
                samples += [(tick+(i+1)*10,0,int(i==0),0) for i in range(8000)]
                result=run(self.executables[0],samples,seed=(0,3.5))[-1]
                self.assertEqual(result[10],0)
                self.assertEqual(result[14],1)
                self.assertEqual(result[1],3.5)


def replay_csv(exes, path, recorded_exe, latest_batch=False):
    rows=[]
    previous_calibration=None
    with path.open(encoding="utf-8-sig",newline="") as stream:
        for row in csv.DictReader(stream):
            if row["kind"]=="angle":
                calibrated=bool(int(row["status_hex"],16)&4)
                if latest_batch and calibrated and previous_calibration is False:
                    rows=[]
                previous_calibration=calibrated
            rows.append(row)
    angles=[r for r in rows if r["kind"]=="angle"]
    raw=[r for r in rows if r["kind"]=="raw"]
    if not raw or len(raw)!=len(angles):
        raise ValueError("Replay requires paired raw and angle records for every sample")
    for a,r in zip(angles,raw):
        if abs(float(a["relative_time_s"])-float(r["relative_time_s"]))>.005:
            raise ValueError("Angle/raw records are not time-aligned")
    for prev,current in zip(raw,raw[1:]):
        if (int(current["sample_tick_ms"])-int(prev["sample_tick_ms"])) & 0xffffffff != 10:
            raise ValueError("This validation requires a complete 100 Hz raw capture")
    begin=next(i for i,a in enumerate(angles) if int(a["status_hex"],16)&4)
    samples=[]
    ends=[]
    legs=0
    first_motion=None
    origin=0
    for i in range(begin,len(raw)):
        state=int(angles[i]["status_hex"],16) & ~0x40
        previous_state=(int(angles[i-1]["status_hex"],16) & ~0x40) if i>begin else state
        if state==7 and previous_state==23 and first_motion is None:
            first_motion=i-begin
        command=2 if (i>begin and state==previous_state==23 and
                      float(angles[i]["angle_deg"])==0 and float(angles[i-1]["angle_deg"])!=0) else 0
        if command and first_motion is not None:
            raise ValueError("Mid-batch zero is not eligible for residual acceptance")
        if command:
            origin=i-begin
        if first_motion is not None and state & 0x25 != 0x05:
            raise ValueError("Invalid/degraded/recalibrating data during the batch")
        samples.append((int(raw[i]["sample_tick_ms"]),int(raw[i]["raw_code"]),command,0))
        if i>begin and state==23 and previous_state==7:
            legs+=1
            if legs%2==0:
                ends.append(i-begin+100)
    if first_motion is None or first_motion==0 or len(ends)!=10 or ends[-1]>=len(samples):
        raise ValueError("Expected ten complete round trips with >=1 s final dwell")
    seed=(float(raw[begin]["bias_dps"]),float(angles[begin]["angle_deg"]))
    output={mode:run(exe,samples,seed) for mode,exe in exes.items()}
    for mode,result in output.items():
        if any(r[4]!=2 or r[6]!=1 or r[7]!=0 or r[10]!=0 for r in result[first_motion:]):
            raise ValueError(f"Replay mode {mode} produced invalid measurement state")
    # The raw CSV bias is rounded to 0.001 dps; don't infer it from target angles.
    recorded=run(recorded_exe,samples,seed)
    error=max(abs(r[1]-float(angles[begin+i]["angle_deg"])) for i,r in enumerate(recorded))
    tolerance=(samples[-1][0]-samples[0][0])*.001*.0005+.005
    if error>tolerance:
        raise ValueError(f"Recorded-mode replay differs by {error:.6f} deg; bound={tolerance:.6f}")
    print(f"Recorded-mode replay max angle difference: {error:.6f} deg (rounded CSV bias)")
    print("Replay uses recorded raw samples; it does not simulate the sensor's internal filter "
          "or verify mechanical position.")
    actual_base=float(angles[begin+origin]["angle_deg"])
    linear_base=output[0][origin][1]
    actual=[float(angles[begin+i]["angle_deg"])-actual_base for i in ends]
    linear=[output[0][i][1]-linear_base for i in ends]
    print("Reference: last explicit zero before motion, or capture start if no zero is present.")
    print("Round,RecordedDeg,ContinuousDeg,PassAbsBelow0.5AtStillPlus1s")
    for i,(a,b) in enumerate(zip(actual,linear),1):
        print(f"{i},{a:.5f},{b:.5f},{abs(b)<.5}")
    print("New precision pass:",sum(abs(x)<.5 for x in linear),"/",len(linear))
    print("New maximum absolute cumulative residual:",max(map(abs,linear)))
    return all(abs(x)<.5 for x in linear)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--csv",type=Path)
    parser.add_argument("--recorded-mode",choices=("legacy","linear-zupt","continuous"),default="legacy")
    parser.add_argument("--latest-batch",action="store_true",
                        help="Explicitly select records after the last calibration completion")
    parser.add_argument("--require-precision",action="store_true")
    args=parser.parse_args()
    if args.require_precision and not args.csv:
        parser.error("--require-precision requires --csv")
    with tempfile.TemporaryDirectory(prefix="gyro-task-replay-") as directory:
        work=Path(directory)
        exes={mode:build(work,mode) for mode in (0,1)}
        GyroTests.executables=exes
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(GyroTests))
        if not result.wasSuccessful():
            raise SystemExit(1)
        if args.csv:
            recorded_exe=exes[1] if args.recorded_mode=="legacy" else (
                exes[0] if args.recorded_mode=="continuous" else build(work,0,1))
            passed=replay_csv(exes,args.csv,recorded_exe,args.latest_batch)
            if args.require_precision and not passed:
                raise SystemExit(2)


if __name__=="__main__":
    main()
