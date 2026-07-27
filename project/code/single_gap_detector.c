/*********************************************************************************************************************
* File: single_gap_detector.c
* Description: Bounded row-run detector for the two-cone validation gate.
*********************************************************************************************************************/

#include "single_gap_detector.h"
#include "single_gap_config.h"

#include <string.h>

#define SINGLE_GAP_FOREGROUND_LEVEL       (160U)
#define SINGLE_GAP_LOCAL_CONTRAST          (40U)
#define SINGLE_GAP_SCORE_ACCEPT            (166U)
#define SINGLE_GAP_INVALID_COORDINATE      (0xFFFFU)

typedef struct
{
    uint16 left;
    uint16 right;
    uint16 label;
} single_gap_run_struct;

typedef struct
{
    uint16 parent;
    uint16 left;
    uint16 right;
    uint16 top;
    uint16 bottom;
    uint16 row_left[SINGLE_GAP_IMAGE_HEIGHT];
    uint16 row_right[SINGLE_GAP_IMAGE_HEIGHT];
    uint32 area;
    uint32 sum_x;
    uint32 sum_value;
    uint8 used;
} single_gap_component_struct;

typedef struct
{
    uint16 bottom_center_x;
    uint16 bottom_y;
    uint16 score;
} single_gap_accepted_struct;

static single_gap_run_struct previous_runs[SINGLE_GAP_MAX_RUNS];
static single_gap_run_struct current_runs[SINGLE_GAP_MAX_RUNS];
static single_gap_component_struct components[SINGLE_GAP_MAX_COMPONENTS];

static uint16 single_gap_min_u16(uint16 first, uint16 second)
{
    return (first < second) ? first : second;
}

static uint16 single_gap_max_u16(uint16 first, uint16 second)
{
    return (first > second) ? first : second;
}

static uint16 single_gap_clamp_score(float value)
{
    if(0.0f >= value)
    {
        return 0U;
    }
    if(256.0f <= value)
    {
        return 256U;
    }
    return (uint16)(value + 0.5f);
}

static void single_gap_components_reset(void)
{
    uint16 component_index;
    uint16 row;

    memset(components, 0, sizeof(components));
    for(component_index = 0U;
        component_index < SINGLE_GAP_MAX_COMPONENTS;
        component_index++)
    {
        components[component_index].parent = component_index;
        components[component_index].left = SINGLE_GAP_IMAGE_WIDTH;
        components[component_index].top = SINGLE_GAP_IMAGE_HEIGHT;
        for(row = 0U; row < SINGLE_GAP_IMAGE_HEIGHT; row++)
        {
            components[component_index].row_left[row] = SINGLE_GAP_INVALID_COORDINATE;
        }
    }
}

static uint16 single_gap_component_find(uint16 label)
{
    uint16 root = label;

    while(components[root].parent != root)
    {
        root = components[root].parent;
    }
    while(components[label].parent != label)
    {
        uint16 parent = components[label].parent;
        components[label].parent = root;
        label = parent;
    }
    return root;
}

static uint16 single_gap_component_create(void)
{
    uint16 index;

    for(index = 0U; index < SINGLE_GAP_MAX_COMPONENTS; index++)
    {
        if(0U == components[index].used)
        {
            components[index].used = 1U;
            components[index].parent = index;
            return index;
        }
    }
    return SINGLE_GAP_MAX_COMPONENTS;
}

static uint16 single_gap_component_union(uint16 first, uint16 second)
{
    uint16 destination;
    uint16 source;
    uint16 row;

    first = single_gap_component_find(first);
    second = single_gap_component_find(second);
    if(first == second)
    {
        return first;
    }

    destination = single_gap_min_u16(first, second);
    source = single_gap_max_u16(first, second);
    components[source].parent = destination;
    components[destination].left = single_gap_min_u16(components[destination].left,
                                                       components[source].left);
    components[destination].right = single_gap_max_u16(components[destination].right,
                                                        components[source].right);
    components[destination].top = single_gap_min_u16(components[destination].top,
                                                      components[source].top);
    components[destination].bottom = single_gap_max_u16(components[destination].bottom,
                                                         components[source].bottom);
    components[destination].area += components[source].area;
    components[destination].sum_x += components[source].sum_x;
    components[destination].sum_value += components[source].sum_value;
    for(row = 0U; row < SINGLE_GAP_IMAGE_HEIGHT; row++)
    {
        if(SINGLE_GAP_INVALID_COORDINATE != components[source].row_left[row])
        {
            if(SINGLE_GAP_INVALID_COORDINATE == components[destination].row_left[row])
            {
                components[destination].row_left[row] = components[source].row_left[row];
                components[destination].row_right[row] = components[source].row_right[row];
            }
            else
            {
                components[destination].row_left[row] =
                    single_gap_min_u16(components[destination].row_left[row],
                                       components[source].row_left[row]);
                components[destination].row_right[row] =
                    single_gap_max_u16(components[destination].row_right[row],
                                       components[source].row_right[row]);
            }
        }
    }
    return destination;
}

static uint8 single_gap_pixel_is_foreground(const uint8 *pixels,
                                             uint16 stride,
                                             uint16 x,
                                             uint16 y)
{
    uint16 first_x = (2U < x) ? (x - 2U) : 0U;
    uint16 last_x = ((x + 2U) < SINGLE_GAP_IMAGE_WIDTH) ?
                    (x + 2U) : (SINGLE_GAP_IMAGE_WIDTH - 1U);
    uint16 sample_x;
    uint16 sample_count = 0U;
    uint16 value = pixels[(uint32)y * stride + x];
    uint32 sum = 0U;

    for(sample_x = first_x; sample_x <= last_x; sample_x++)
    {
        sum += pixels[(uint32)y * stride + sample_x];
        sample_count++;
    }
    return ((SINGLE_GAP_FOREGROUND_LEVEL <= value) ||
            (((sum / sample_count) + SINGLE_GAP_LOCAL_CONTRAST) <= value)) ? 1U : 0U;
}

static uint8 single_gap_image_quality_valid(const uint8 *pixels, uint16 stride)
{
    uint16 minimum = 255U;
    uint16 maximum = 0U;
    uint16 value;
    uint16 x;
    uint16 y;
    uint32 bright_count = 0U;
    uint32 dark_count = 0U;
    uint32 sample_count = 0U;

    for(y = 0U; y < SINGLE_GAP_IMAGE_HEIGHT; y += 4U)
    {
        for(x = 0U; x < SINGLE_GAP_IMAGE_WIDTH; x += 4U)
        {
            value = pixels[(uint32)y * stride + x];
            minimum = single_gap_min_u16(minimum, value);
            maximum = single_gap_max_u16(maximum, value);
            dark_count += (5U >= value) ? 1U : 0U;
            bright_count += (250U <= value) ? 1U : 0U;
            sample_count++;
        }
    }

    return (((uint16)(maximum - minimum) >= 24U) &&
            ((dark_count * 10U) <= (sample_count * 3U)) &&
            ((bright_count * 10U) <= (sample_count * 3U))) ? 1U : 0U;
}

static void single_gap_component_add_run(uint16 label,
                                          const uint8 *pixels,
                                          uint16 stride,
                                          uint16 y,
                                          uint16 left,
                                          uint16 right)
{
    uint16 x;
    single_gap_component_struct *component;

    label = single_gap_component_find(label);
    component = &components[label];
    component->left = single_gap_min_u16(component->left, left);
    component->right = single_gap_max_u16(component->right, right);
    component->top = single_gap_min_u16(component->top, y);
    component->bottom = single_gap_max_u16(component->bottom, y);
    if(SINGLE_GAP_INVALID_COORDINATE == component->row_left[y])
    {
        component->row_left[y] = left;
        component->row_right[y] = right;
    }
    else
    {
        component->row_left[y] = single_gap_min_u16(component->row_left[y], left);
        component->row_right[y] = single_gap_max_u16(component->row_right[y], right);
    }
    for(x = left; x <= right; x++)
    {
        component->area++;
        component->sum_x += x;
        component->sum_value += pixels[(uint32)y * stride + x];
    }
}

static uint16 single_gap_component_row_width(const single_gap_component_struct *component,
                                              uint16 row)
{
    if((SINGLE_GAP_IMAGE_HEIGHT <= row) ||
       (SINGLE_GAP_INVALID_COORDINATE == component->row_left[row]))
    {
        return 0U;
    }
    return component->row_right[row] - component->row_left[row] + 1U;
}

static uint16 single_gap_component_score(const single_gap_component_struct *component,
                                          const uint8 *pixels,
                                          uint16 stride)
{
    static const uint16 slice_percent[5] = {15U, 30U, 50U, 70U, 85U};
    uint16 slice_width[5];
    uint16 maximum_slice_width = 1U;
    uint16 band_top;
    uint16 band_bottom;
    uint16 outer_left;
    uint16 outer_right;
    uint16 outer_top;
    uint16 outer_bottom;
    uint16 height;
    uint16 width;
    uint16 row;
    uint16 x;
    uint16 index;
    uint32 band_count = 0U;
    uint32 band_sum = 0U;
    uint32 border_count = 0U;
    uint32 border_sum = 0U;
    float aspect_ratio;
    float aspect_error;
    float background_mean;
    float band_mean;
    float bbox_center;
    float centroid;
    float foreground_mean;
    uint16 taper;
    uint16 band;
    uint16 symmetry;
    uint16 aspect;
    uint16 base;
    uint16 contrast;

    height = component->bottom - component->top + 1U;
    width = component->right - component->left + 1U;
    for(index = 0U; index < 5U; index++)
    {
        row = component->top + (uint16)(((uint32)(height - 1U) * slice_percent[index] + 50U) / 100U);
        slice_width[index] = single_gap_component_row_width(component, row);
        maximum_slice_width = single_gap_max_u16(maximum_slice_width, slice_width[index]);
    }

    taper = (slice_width[4] > slice_width[0]) ?
             single_gap_clamp_score(256.0f * (float)(slice_width[4] - slice_width[0]) /
                                    (float)single_gap_max_u16(1U, slice_width[4])) : 0U;

    band_top = component->top + (uint16)(((uint32)height * 30U) / 100U);
    band_bottom = component->top + (uint16)(((uint32)height * 60U) / 100U);
    for(row = band_top; (row <= band_bottom) && (row < SINGLE_GAP_IMAGE_HEIGHT); row++)
    {
        if(SINGLE_GAP_INVALID_COORDINATE != component->row_left[row])
        {
            for(x = component->row_left[row]; x <= component->row_right[row]; x++)
            {
                band_sum += pixels[(uint32)row * stride + x];
                band_count++;
            }
        }
    }
    band_mean = (0U != band_count) ? ((float)band_sum / (float)band_count) : 0.0f;
    band = single_gap_clamp_score((band_mean - 180.0f) * 256.0f / 75.0f);

    centroid = (float)component->sum_x / (float)component->area;
    bbox_center = 0.5f * (float)(component->left + component->right);
    symmetry = single_gap_clamp_score(256.0f *
               (1.0f - ((centroid > bbox_center) ? (centroid - bbox_center) :
                                                      (bbox_center - centroid)) /
                       single_gap_max_u16(1U, width / 2U)));

    aspect_ratio = (float)height / (float)width;
    aspect_error = (aspect_ratio > 1.3f) ? (aspect_ratio - 1.3f) : (1.3f - aspect_ratio);
    aspect = single_gap_clamp_score(256.0f * (1.0f - aspect_error / 1.3f));
    base = single_gap_clamp_score(256.0f * (float)slice_width[4] /
                                 (float)maximum_slice_width);

    outer_left = (2U < component->left) ? (component->left - 2U) : 0U;
    outer_right = ((component->right + 2U) < SINGLE_GAP_IMAGE_WIDTH) ?
                  (component->right + 2U) : (SINGLE_GAP_IMAGE_WIDTH - 1U);
    outer_top = (2U < component->top) ? (component->top - 2U) : 0U;
    outer_bottom = ((component->bottom + 2U) < SINGLE_GAP_IMAGE_HEIGHT) ?
                    (component->bottom + 2U) : (SINGLE_GAP_IMAGE_HEIGHT - 1U);
    for(x = outer_left; x <= outer_right; x++)
    {
        border_sum += pixels[(uint32)outer_top * stride + x];
        border_sum += pixels[(uint32)outer_bottom * stride + x];
        border_count += 2U;
    }
    for(row = (uint16)(outer_top + 1U); row < outer_bottom; row++)
    {
        border_sum += pixels[(uint32)row * stride + outer_left];
        border_sum += pixels[(uint32)row * stride + outer_right];
        border_count += 2U;
    }
    background_mean = (0U != border_count) ? ((float)border_sum / (float)border_count) : 0.0f;
    foreground_mean = (float)component->sum_value / (float)component->area;
    contrast = single_gap_clamp_score((foreground_mean - background_mean) * 256.0f / 100.0f);

    return (uint16)((77UL * taper + 64UL * band + 38UL * symmetry +
                     26UL * aspect + 26UL * base + 25UL * contrast + 128UL) >> 8U);
}

static void single_gap_sort_accepted(single_gap_accepted_struct *accepted, uint16 count)
{
    uint16 first;
    uint16 second;

    for(first = 0U; first < count; first++)
    {
        for(second = (uint16)(first + 1U); second < count; second++)
        {
            if(accepted[second].bottom_center_x < accepted[first].bottom_center_x)
            {
                single_gap_accepted_struct temporary = accepted[first];
                accepted[first] = accepted[second];
                accepted[second] = temporary;
            }
        }
    }
}

uint8 single_gap_detector_process(const uint8 *pixels,
                                  uint16 width,
                                  uint16 height,
                                  uint16 stride,
                                  uint32 sequence,
                                  uint32 capture_ms,
                                  single_gap_observation_struct *observation)
{
    single_gap_accepted_struct accepted[SINGLE_GAP_MAX_ACCEPTED];
    uint16 accepted_count = 0U;
    uint16 component_count = 0U;
    uint16 current_count;
    uint16 previous_count = 0U;
    uint16 component_index;
    uint16 overlap_index;
    uint16 label;
    uint16 left;
    uint16 right;
    uint16 x;
    uint16 y;
    uint8 capacity_overflow = 0U;

    if(NULL == observation)
    {
        return 0U;
    }
    memset(observation, 0, sizeof(*observation));
    if((NULL == pixels) || (SINGLE_GAP_IMAGE_WIDTH != width) ||
       (SINGLE_GAP_IMAGE_HEIGHT != height) || (width > stride))
    {
        return 0U;
    }
    observation->sequence = sequence;
    observation->capture_ms = capture_ms;
    if(0U == single_gap_image_quality_valid(pixels, stride))
    {
        return 1U;
    }

    single_gap_components_reset();
    for(y = SINGLE_GAP_ROI_TOP_PX; y <= SINGLE_GAP_ROI_BOTTOM_PX; y++)
    {
        current_count = 0U;
        x = 0U;
        while(x < SINGLE_GAP_IMAGE_WIDTH)
        {
            if(0U == single_gap_pixel_is_foreground(pixels, stride, x, y))
            {
                x++;
                continue;
            }
            left = x;
            while(((x + 1U) < SINGLE_GAP_IMAGE_WIDTH) &&
                  (0U != single_gap_pixel_is_foreground(pixels, stride, (uint16)(x + 1U), y)))
            {
                x++;
            }
            right = x;
            label = SINGLE_GAP_MAX_COMPONENTS;
            for(overlap_index = 0U; overlap_index < previous_count; overlap_index++)
            {
                if((previous_runs[overlap_index].left <= right) &&
                   (previous_runs[overlap_index].right >= left))
                {
                    if(SINGLE_GAP_MAX_COMPONENTS == label)
                    {
                        label = single_gap_component_find(previous_runs[overlap_index].label);
                    }
                    else
                    {
                        label = single_gap_component_union(label,
                            previous_runs[overlap_index].label);
                    }
                }
            }
            if(SINGLE_GAP_MAX_COMPONENTS == label)
            {
                label = single_gap_component_create();
                if(SINGLE_GAP_MAX_COMPONENTS != label)
                {
                    component_count++;
                }
            }
            if((SINGLE_GAP_MAX_COMPONENTS == label) ||
               (SINGLE_GAP_MAX_RUNS <= current_count))
            {
                capacity_overflow = 1U;
            }
            else
            {
                single_gap_component_add_run(label, pixels, stride, y, left, right);
                current_runs[current_count].left = left;
                current_runs[current_count].right = right;
                current_runs[current_count].label = single_gap_component_find(label);
                current_count++;
            }
            x++;
        }
        memcpy(previous_runs, current_runs, current_count * sizeof(current_runs[0]));
        previous_count = current_count;
    }

    for(component_index = 0U;
        component_index < SINGLE_GAP_MAX_COMPONENTS;
        component_index++)
    {
        single_gap_component_struct *component = &components[component_index];
        uint16 component_height;
        uint16 component_width;
        uint16 score;

        if((0U == component->used) ||
           (component_index != single_gap_component_find(component_index)) ||
           (0U == component->area))
        {
            continue;
        }
        component_height = component->bottom - component->top + 1U;
        component_width = component->right - component->left + 1U;
        if((8U > component_height) || (4U > component_width) || (24U > component->area))
        {
            continue;
        }
        score = single_gap_component_score(component, pixels, stride);
        if(SINGLE_GAP_SCORE_ACCEPT <= score)
        {
            if(SINGLE_GAP_MAX_ACCEPTED > accepted_count)
            {
                accepted[accepted_count].bottom_center_x =
                    (uint16)((component->left + component->right) / 2U);
                accepted[accepted_count].bottom_y = component->bottom;
                accepted[accepted_count].score = score;
                accepted_count++;
            }
            else
            {
                capacity_overflow = 1U;
            }
        }
    }

    (void)component_count;
    if(0U != capacity_overflow)
    {
        observation->accepted_count = SINGLE_GAP_MAX_ACCEPTED;
        observation->ambiguous = 1U;
        return 1U;
    }
    observation->accepted_count = accepted_count;
    if(SINGLE_GAP_MAX_ACCEPTED == accepted_count)
    {
        observation->ambiguous = 1U;
        return 1U;
    }
    if(2U != accepted_count)
    {
        return 1U;
    }

    single_gap_sort_accepted(accepted, accepted_count);
    observation->gap_width_px = accepted[1].bottom_center_x - accepted[0].bottom_center_x;
    observation->gap_center_x =
        (uint16)((accepted[0].bottom_center_x + accepted[1].bottom_center_x) / 2U);
    observation->left_bottom_y = accepted[0].bottom_y;
    observation->right_bottom_y = accepted[1].bottom_y;
    if(SINGLE_GAP_MIN_WIDTH_PX <= observation->gap_width_px)
    {
        observation->valid = 1U;
    }
    return 1U;
}
