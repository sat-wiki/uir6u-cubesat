*** Settings ***
Documentation       Boot the production STM32H743 ELF with the board RF chips and NAND absent.
...                 Run with Renode 1.17 renode-test and its Python test requirements.
...                 Build the Debug preset first, or pass --variable ELF:/path/to/cubesat.elf.
Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Boot Board
Test Teardown       Test Teardown

*** Variables ***
${ELF}              ${CURDIR}/../../build/Debug/cubesat.elf

*** Keywords ***
Boot Board
    Test Setup
    Execute Command    mach create
    ${board}=    Replace String    ${CURDIR}/board.repl    ${SPACE}    \\${SPACE}
    ${image}=    Replace String    ${ELF}    ${SPACE}    \\${SPACE}
    Execute Command    machine LoadPlatformDescription @${board}
    Execute Command    sysbus LoadELF @${image}
    Execute Command    emulation CreateCANHub "busA"
    Execute Command    emulation CreateCANHub "busB"
    Execute Command    connector Connect fdcan1 busA
    Execute Command    connector Connect fdcan2 busB
    Execute Command    gpioPortE.powerGood Press
    Execute Command    gpioPortC.txLocked Press
    ${pa}=    Create LED Tester    sysbus.gpioPortC.paShutdown
    ${wdi}=    Create LED Tester    sysbus.gpioPortD.watchdogInput
    Set Test Variable    ${PA}    ${pa}
    Set Test Variable    ${WDI}    ${wdi}
    Execute Command    logLevel 3

Read Firmware Word
    [Arguments]    ${symbol}
    ${address}=    Execute Command    sysbus GetSymbolAddress "${symbol}"
    ${value}=      Execute Command    sysbus ReadDoubleWord ${address.strip()}
    RETURN    ${value}

Send Extended CAN Frame
    [Arguments]    ${controller}    ${id}    ${bytes}
    Execute Command    python "from System import Array, Byte; from Antmicro.Renode.Core.CAN import CANMessageFrame; self.Machine['sysbus.${controller}'].OnFrameReceived(CANMessageFrame(${id}, Array[Byte]([${bytes}]), True, False, False, False))"

*** Test Cases ***
Boot Scheduler And Service Watchdog
    Execute Command    emulation RunFor "0.35"
    ${scheduler}=    Read Firmware Word    xSchedulerRunning
    Should Be Equal As Integers    ${scheduler}    1
    ${tasks}=    Read Firmware Word    uxCurrentNumberOfTasks
    Should Be Equal As Integers    ${tasks}    4
    ${old}=    Read Firmware Word    heartbeat
    Execute Command    emulation RunFor "0.13"
    ${new}=    Read Firmware Word    heartbeat
    Should Be True    int($new, 0) > int($old, 0) > 0
    Assert LED Is Blinking    testDuration=0.45    onDuration=0.1    offDuration=0.1    tolerance=0.01    testerId=${WDI}
    Assert LED State    true    timeout=0    testerId=${PA}

Report Lock And Power Inputs With RF Unavailable
    ${can}=    Create CAN Tester    busA
    ${canB}=    Create CAN Tester    busB
    ${frame}=    Wait For Frame With Id    0x12A30000    timeout=1    testerId=${can}
    ${copy}=    Wait For Frame With Id    0x12A30000    timeout=1    testerId=${canB}
    Should Be Equal    ${frame}    ${copy}
    Length Should Be    ${frame}    8
    # CAN1, CAN2 and TX lock are up. RF init and NAND availability are clear.
    Should Be True    $frame[7] & 0x4f == 0x43
    Should Be True    not (int.from_bytes($frame[4:6], 'big') & 0x10)
    Execute Command    gpioPortC.txLocked Release
    Execute Command    gpioPortE.powerGood Release
    Execute Command    emulation RunFor "1.2"
    ${faults}=    Read Firmware Word    faults
    Should Be True    int($faults, 0) & 0x10 != 0
    Assert And Hold LED State    true    timeoutAssert=0    timeoutHold=0.25    testerId=${PA}

Handle CAN Time Transfer On Either Bus
    Execute Command    emulation RunFor "0.35"
    FOR    ${bus}    ${sequence}    ${marker}    IN
    ...    fdcan1    0x12a70001    0x21
    ...    fdcan2    0x12a70002    0x32
        Send Extended CAN Frame    ${bus}    ${sequence}    ${EMPTY}
        Execute Command    emulation RunFor "0.005"
        Send Extended CAN Frame    ${bus}    ${sequence}    ${marker},0,0,0
        Execute Command    emulation RunFor "0.010"
        ${stamp}=    Read Firmware Word    spacecraft_us
        Should Be True    int($stamp, 0) >> 24 == int($marker, 0)
    END
    # A valid lease and telemetry cannot enable the PA without the RF and NAND devices.
    Send Extended CAN Frame    fdcan1    0x12a40003    1,60
    Send Extended CAN Frame    fdcan1    0x12a20004    1,2,3
    Assert And Hold LED State    true    timeoutAssert=0    timeoutHold=0.3    testerId=${PA}
