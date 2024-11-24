#include "gtest/gtest.h"
#include "N_USData_Request_Runner.h"
#include "N_USData_Indication_Runner.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"

static LinuxOSShim linuxOSShim;

TEST(N_USDATA_RUNNER, constructor_argument_availableMemoryTest)
{
    LocalCANNetwork can_network;
    uint32_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    int messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, *can_network.newCANShimConnection());

    int64_t actualMemory;
    ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
    ASSERT_EQ(availableMemoryConst, actualMemory + messageLen);
    ASSERT_TRUE(result);
}