#include <stdio.h>
#include <string.h>

#define CAMERA_SEEKFREE_TRANSPORT_HOST_TEST
#include "../project/code/camera_seekfree_transport.c"

seekfree_assistant_transfer_callback_function seekfree_assistant_transfer_callback;

static uint32 mock_remaining[4];
static uint32 mock_call_count;
static int mock_interface;

void seekfree_assistant_interface_init(int device)
{
    mock_interface = device;
}

uint32 wifi_spi_send_buffer(const uint8 *buffer, uint32 length)
{
    uint32 result = mock_remaining[mock_call_count];
    (void)buffer;
    (void)length;
    mock_call_count++;
    return result;
}

static void reset_mock(void)
{
    memset(mock_remaining, 0, sizeof(mock_remaining));
    mock_call_count = 0U;
}

#define CHECK(condition, message) \
    do { if(!(condition)) { fprintf(stderr, "FAIL: %s\n", message); return 1; } } while(0)

int main(void)
{
    uint8 header[12] = {0U};
    uint8 payload[32] = {0U};
    const camera_seekfree_transport_diag_struct *diag;

    camera_seekfree_transport_install();
    CHECK(SEEKFREE_ASSISTANT_CUSTOM == mock_interface, "custom interface not selected");
    CHECK(camera_seekfree_transport_transfer == seekfree_assistant_transfer_callback,
          "transfer callback not installed");

    reset_mock();
    camera_seekfree_transport_begin(sizeof(header), sizeof(payload));
    CHECK(0U == seekfree_assistant_transfer_callback(header, sizeof(header)), "full header");
    CHECK(0U == seekfree_assistant_transfer_callback(payload, sizeof(payload)), "full payload");
    CHECK(1U == camera_seekfree_transport_frame_complete(), "full frame rejected");
    CHECK(2U == mock_call_count, "full frame call count");

    reset_mock();
    mock_remaining[0] = 4U;
    camera_seekfree_transport_begin(sizeof(header), sizeof(payload));
    CHECK(4U == seekfree_assistant_transfer_callback(header, sizeof(header)), "partial header");
    CHECK(sizeof(payload) == seekfree_assistant_transfer_callback(payload, sizeof(payload)),
          "payload after partial header was not rejected");
    CHECK(1U == mock_call_count, "payload reached WiFi after partial header");
    CHECK(0U == camera_seekfree_transport_frame_complete(), "partial header accepted");
    diag = camera_seekfree_transport_get_diag();
    CHECK(sizeof(payload) == diag->payload_remaining, "payload remainder not recorded");

    reset_mock();
    mock_remaining[1] = 7U;
    camera_seekfree_transport_begin(sizeof(header), sizeof(payload));
    (void)seekfree_assistant_transfer_callback(header, sizeof(header));
    CHECK(7U == seekfree_assistant_transfer_callback(payload, sizeof(payload)), "partial payload");
    CHECK(0U == camera_seekfree_transport_frame_complete(), "partial payload accepted");

    reset_mock();
    camera_seekfree_transport_begin(sizeof(header), sizeof(payload));
    (void)seekfree_assistant_transfer_callback(header, sizeof(header));
    (void)seekfree_assistant_transfer_callback(payload, sizeof(payload));
    CHECK(1U == camera_seekfree_transport_frame_complete(), "pre-third complete frame rejected");
    CHECK(1U == seekfree_assistant_transfer_callback(payload, 1U), "third segment not rejected");
    CHECK(2U == mock_call_count, "unexpected third segment reached WiFi");
    CHECK(0U == camera_seekfree_transport_frame_complete(), "third segment accepted");

    reset_mock();
    camera_seekfree_transport_begin(sizeof(header), sizeof(payload));
    (void)seekfree_assistant_transfer_callback(header, sizeof(header) - 1U);
    (void)seekfree_assistant_transfer_callback(payload, sizeof(payload));
    CHECK(0U == camera_seekfree_transport_frame_complete(), "length mismatch accepted");

    camera_seekfree_transport_begin(sizeof(header), sizeof(payload));
    diag = camera_seekfree_transport_get_diag();
    CHECK((0U == diag->segment_count) && (0U == diag->header_remaining) &&
          (0U == diag->payload_remaining), "begin did not reset attempt state");

    puts("PASS: camera Seekfree transport host unit test");
    return 0;
}
