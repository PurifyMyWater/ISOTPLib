#ifndef CANShim_h
#define CANShim_h

#include <cstdint>

#define CAN_FRAME_MAX_DLC 8


typedef enum N_TAtype
{
    CAN_UNKNOWN = 0,
    CAN_CLASSIC_29bit_Physical = 218,
    CAN_CLASSIC_29bit_Functional = 219
} N_TAtype_t;

typedef union N_AI
{
    struct __attribute__((packed))
    {
        uint8_t : 3, N_NFA_Header : 3 {0b110}, N_NFA_Padding : 2 {0b00};
        N_TAtype_t N_TAtype : 8 {CAN_UNKNOWN};
        uint8_t N_TA {0};
        uint8_t N_SA {0};
    };
    uint32_t N_AI;
} N_AI;

typedef struct CANFrame
{
    union
    {
        struct
        {
            uint32_t extd: 1;           /**< Extended Frame Format (29bit ID) */
            uint32_t rtr: 1;            /**< Message is a Remote Frame */
            uint32_t ss: 1;             /**< Transmit as a Single Shot Transmission. Unused for received. */
            uint32_t self: 1;           /**< Transmit as a Self Reception Request. Unused for received. */
            uint32_t dlc_non_comp: 1;   /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */
            uint32_t reserved: 27;      /**< Reserved bits */
        };
        uint32_t flags;                 /**< Deprecated: Alternate way to set bits using message flags */
    };
    N_AI identifier;                /**< 11 or 29 bit identifier */
    uint8_t data_length_code;           /**< Data length code */
    uint8_t data[CAN_FRAME_MAX_DLC];    /**< Data bytes (not relevant in RTR frame) */
} CANFrame;

/**
 * @brief Interface for a CAN bus driver
 */
class CANShim
{
public:

    /**
     * @brief Check if a frame is available to read
     * @return Number of frames available to read
     */
    virtual uint32_t frameAvailable() = 0;

    /**
     * @brief Read a frame from the CAN bus
     * @param frame Pointer to a CANFrame struct to store the read frame
     * @return True if a frame was read, false if no frame was available
     */
    virtual bool readFrame(CANFrame* frame) = 0;

    /**
     * @brief Write a frame to the CAN bus
     * @param frame Pointer to a CANFrame struct to write to the bus
     * @return True if the frame was written, false if the bus is not active or the frame was not written
     */
    virtual bool writeFrame(CANFrame* frame) = 0;

    /**
     * @brief Check if the bus is active
     * @return True if the bus is active, false if the bus is not active
     */
    virtual bool active() = 0;

    virtual ~CANShim() = default;
};

#endif // CANShim_h
