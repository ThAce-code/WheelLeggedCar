#ifndef _perception_intercore_h_
#define _perception_intercore_h_

#include "intercore_protocol.h"
#include "intercore_transport.h"

typedef struct
{
    volatile intercore_shared_layout_struct *shared;
    uint32 boot_epoch;
    uint32 last_pose_sequence;
    uint32 last_perception_sequence;
    uint8 role;
    uint8 attached;
}perception_intercore_transport_struct;

uint8 perception_intercore_cm7_0_init(perception_intercore_transport_struct *transport,
                                      volatile intercore_shared_layout_struct *shared);
uint8 perception_intercore_cm7_1_attach(perception_intercore_transport_struct *transport,
                                        volatile intercore_shared_layout_struct *shared);
uint8 perception_intercore_publish_pose(perception_intercore_transport_struct *transport,
                                        const perception_pose_snapshot_struct *snapshot);
uint8 perception_intercore_read_pose(perception_intercore_transport_struct *transport,
                                     perception_pose_snapshot_struct *snapshot);
uint8 perception_intercore_publish_perception(
    perception_intercore_transport_struct *transport,
    const perception_snapshot_struct *snapshot);
uint8 perception_intercore_read_perception(
    perception_intercore_transport_struct *transport,
    perception_snapshot_struct *snapshot);

#endif
