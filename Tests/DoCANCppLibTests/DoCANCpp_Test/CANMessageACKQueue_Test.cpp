#include "CANMessageACKQueue.h"

#include <DoCANCpp.h>
#include <N_USData_Request_Runner.h>

#include <N_USData_Runner.h>
#include "ASSERT_MACROS.h"
#include "LinuxOSInterface.h"
#include "LocalCANNetwork.h"
#include "gtest/gtest.h"

static LinuxOSInterface linuxOSInterface;

TEST(CANMessageACKQueue, writeFrame)
{
    // Given
    LocalCANNetwork    localCANNetwork;
    CANInterface*      canInterface = localCANNetwork.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);
    CANFrame           frame = NewCANFrameDoCANCpp();

    // Create dumb runner
    int64_t                 availableMemoryConst = 100;
    Atomic_int64_t          availableMemoryMock(availableMemoryConst, linuxOSInterface);
    N_AI                    NAi               = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);
    const char*             testMessageString = ""; // strlen = 0
    size_t                  messageLen        = strlen(testMessageString);
    const uint8_t*          testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool                    result;
    N_USData_Request_Runner runner(result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen,
                                   linuxOSInterface, canMessageACKQueue);

    for (int i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    CANInterface* receivedCanInterface = localCANNetwork.newCANInterfaceConnection();

    bool res = canMessageACKQueue.writeFrame(runner, frame);

    // Want
    bool     expected_result = true;
    CANFrame realFrame;
    receivedCanInterface->readFrame(&realFrame);

    ASSERT_EQ(expected_result, res);
    ASSERT_EQ_FRAMES(frame, realFrame);

    delete canInterface;
    delete receivedCanInterface;
}
