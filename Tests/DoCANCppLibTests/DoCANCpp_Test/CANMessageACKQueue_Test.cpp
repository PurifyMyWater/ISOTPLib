#include "CANMessageACKQueue.h"

#include <DoCANCpp.h>
#include <N_USData_Request_Runner.h>

#include <N_USData_Runner.h>
#include "LocalCANNetwork.h"
#include "gtest/gtest.h"
#include "LinuxOSShim.h"
#include "ASSERT_MACROS.h"

static LinuxOSShim linuxOSShim;

TEST(CANMessageACKQueue, writeFrame)
{
    // Given
    LocalCANNetwork localCANNetwork;
    CANShim* canShim = localCANNetwork.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);
    CANFrame frame = NewCANFrameDoCANCpp();

    // Create dumb runner
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;
    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

    for (int i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    CANShim* receivedCanShim = localCANNetwork.newCANShimConnection();

    bool res = canMessageACKQueue.writeFrame(runner, frame);

    // Want
    bool expected_result = true;
    CANFrame realFrame;
    receivedCanShim->readFrame(&realFrame);

    ASSERT_EQ(expected_result, res);
    ASSERT_EQ_FRAMES(frame, realFrame);
}
