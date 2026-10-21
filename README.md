# Project1_10_13

STM32F103/XV7001BB firmware and the matching Windows desktop CAN application.
This source snapshot was uploaded from the local `project_10_13` project on
2026-10-21. The original firmware history is retained; firmware files now live
under `Project1_10_13/` so the repository matches the local project layout.

## Projects

- [Firmware source and operating notes](Project1_10_13/README.md)
- [Visual Studio / VisualGDB solution](Project1_10_13/Project1_10_13.sln)
- [Firmware main source](Project1_10_13/Project1_10_13.c)
- [Windows desktop application and instructions](host_app/README.md)
- [Desktop application project](host_app/src/GyroHost.App/GyroHost.App.csproj)
- [Desktop application tests](host_app/tests/GyroHost.Tests/GyroHost.Tests.csproj)

The root Windows batch files launch or publish the desktop application.
Building it requires the .NET 10 SDK. Firmware development uses the existing
VisualGDB/Arm toolchain configuration; see the firmware instructions.

Local caches, compiled binaries, published applications, duplicate backups,
personal IDE settings, and credentials are not included.
This is a source backup, not a full flash image backup.

The separate `gyro_project` browser-based handoff is not included here.
Its `0x600/0x601` sensor protocol is different from this project's
`0x321/0x323` protocol. Uploading these files does not flash hardware or
validate motor operation, calibration quality, or physical accuracy.
