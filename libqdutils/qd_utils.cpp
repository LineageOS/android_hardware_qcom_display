/*
 * Copyright (c) 2013, 2017 The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <unistd.h>
#include "qd_utils.h"

namespace qdutils {

static int parseLine(char *input, char *tokens[], const uint32_t maxToken, uint32_t *count) {
    char *tmpToken = NULL;
    char *tmpPtr;
    uint32_t index = 0;
    const char *delim = ", =\n";
    if (!input) {
      return -1;
    }
    tmpToken = strtok_r(input, delim, &tmpPtr);
    while (tmpToken && index < maxToken) {
      tokens[index++] = tmpToken;
      tmpToken = strtok_r(NULL, delim, &tmpPtr);
    }
    *count = index;

    return 0;
}

static int getExternalNode(const char *type) {
    FILE *displayDeviceFP = NULL;
    char fbType[MAX_FRAME_BUFFER_NAME_SIZE];
    char msmFbTypePath[MAX_FRAME_BUFFER_NAME_SIZE];
    int j = 0;

    for(j = 0; j < HWC_NUM_DISPLAY_TYPES; j++) {
        snprintf (msmFbTypePath, sizeof(msmFbTypePath),
                  "/sys/class/graphics/fb%d/msm_fb_type", j);
        displayDeviceFP = fopen(msmFbTypePath, "r");
        if(displayDeviceFP) {
            fread(fbType, sizeof(char), MAX_FRAME_BUFFER_NAME_SIZE,
                    displayDeviceFP);
            if(strncmp(fbType, type, strlen(type)) == 0) {
                ALOGD("%s: %s is at fb%d", __func__, type, j);
                fclose(displayDeviceFP);
                break;
            }
            fclose(displayDeviceFP);
        } else {
            ALOGE("%s: Failed to open fb node %d", __func__, j);
        }
    }

    if (j < HWC_NUM_DISPLAY_TYPES)
        return j;
    else
        ALOGE("%s: Failed to find %s node", __func__, type);

    return -1;
}

static int querySDEInfoDRM(HWQueryType type, int *value) {
    char property[PROPERTY_VALUE_MAX] = {0};

    // TODO(user): If future targets don't support WB UBWC, add separate
    // properties in target specific system.prop and have clients like WFD
    // directly rely on those.
    switch(type) {
    case HAS_UBWC:
    case HAS_WB_UBWC:  // WFD stack still uses this
        *value = 1;
        property_get("debug.gralloc.gfx_ubwc_disable", property, "0");
        if(!(strncmp(property, "1", PROPERTY_VALUE_MAX)) ||
                !(strncmp(property, "true", PROPERTY_VALUE_MAX))) {
            *value = 0;
        }
        break;
    default:
        ALOGE("Invalid query type %d", type);
        return -EINVAL;
    }

    return 0;
}

static int querySDEInfoFB(HWQueryType type, int *value) {
    FILE *fileptr = NULL;
    const char *featureName;
    char stringBuffer[MAX_STRING_LENGTH];
    uint32_t tokenCount = 0;
    const uint32_t maxCount = 10;
    char *tokens[maxCount] = { NULL };

    switch(type) {
    case HAS_UBWC:
        featureName = "ubwc";
        break;
    case HAS_WB_UBWC:
        featureName = "wb_ubwc";
        break;
    default:
        ALOGE("Invalid query type %d", type);
        return -EINVAL;
    }

    fileptr = fopen("/sys/devices/virtual/graphics/fb0/mdp/caps", "rb");
    if (!fileptr) {
        ALOGE("File '%s' not found", stringBuffer);
        return -EINVAL;
    }

    size_t len = MAX_STRING_LENGTH;
    ssize_t read;
    char *line = stringBuffer;
    while ((read = getline(&line, &len, fileptr)) != -1) {
        // parse the line and update information accordingly
        if (parseLine(line, tokens, maxCount, &tokenCount)) {
            continue;
        }

        if (strncmp(tokens[0], "features", strlen("features"))) {
            continue;
        }

        for (uint32_t i = 0; i < tokenCount; i++) {
            if (!strncmp(tokens[i], featureName, strlen(featureName))) {
              *value = 1;
            }
        }
    }
    fclose(fileptr);

    return 0;
}

int querySDEInfo(HWQueryType type, int *value) {
    if (!value) {
        return -EINVAL;
    }

    if (getDriverType() ==  DriverType::DRM) {
        return querySDEInfoDRM(type, value);
    }

    return querySDEInfoFB(type, value);
}

int getHDMINode(void) {
    return getExternalNode("dtv panel");
}

int getEdidRawData(char *buffer)
{
    int size;
    int edidFile;
    char msmFbTypePath[MAX_FRAME_BUFFER_NAME_SIZE];
    int node_id = getHDMINode();

    if (node_id < 0) {
        ALOGE("%s no HDMI node found", __func__);
        return 0;
    }

    snprintf(msmFbTypePath, sizeof(msmFbTypePath),
                 "/sys/class/graphics/fb%d/edid_raw_data", node_id);

    edidFile = open(msmFbTypePath, O_RDONLY, 0);

    if (edidFile < 0) {
        ALOGE("%s no edid raw data found", __func__);
        return 0;
    }

    size = (int)read(edidFile, (char*)buffer, EDID_RAW_DATA_SIZE);
    close(edidFile);
    return size;
}

bool isDPConnected() {
    char connectPath[MAX_FRAME_BUFFER_NAME_SIZE];
    FILE *connectFile = NULL;
    size_t len = MAX_STRING_LENGTH;
    char stringBuffer[MAX_STRING_LENGTH];
    char *line = stringBuffer;

    int nodeId = getExternalNode("dp panel");
    if (nodeId < 0) {
        ALOGE("%s no DP node found", __func__);
        return false;
    }

    snprintf(connectPath, sizeof(connectPath),
             "/sys/class/graphics/fb%d/connected", nodeId);

    connectFile = fopen(connectPath, "rb");
    if (!connectFile) {
        ALOGW("Failed to open connect node for device node %d", nodeId);
        return false;
    }

    if (getline(&line, &len, connectFile) < 0) {
        fclose(connectFile);
        return false;
    }

    fclose(connectFile);

    return atoi(line);
}

int getDPTestConfig(uint32_t *panelBpp, uint32_t *patternType) {
    if (!panelBpp || !patternType) {
        return -1;
    }

    char configPath[MAX_FRAME_BUFFER_NAME_SIZE];
    FILE *configFile = NULL;
    uint32_t tokenCount = 0;
    const uint32_t maxCount = 10;
    char *tokens[maxCount] = { NULL };
    size_t len = MAX_STRING_LENGTH;
    char stringBuffer[MAX_STRING_LENGTH];
    char *line = stringBuffer;

    int nodeId = getExternalNode("dp panel");
    if (nodeId < 0) {
        ALOGE("%s no DP node found", __func__);
        return -EINVAL;
    }

    snprintf(configPath, sizeof(configPath),
             "/sys/class/graphics/fb%d/config", nodeId);

    configFile = fopen(configPath, "rb");
    if (!configFile) {
        ALOGW("Failed to open config node for device node %d", nodeId);
        return -EINVAL;
    }

    while (getline(&line, &len, configFile) != -1) {
        if (!parseLine(line, tokens, maxCount, &tokenCount)) {
            if (!strncmp(tokens[0], "bpp", strlen("bpp"))) {
                *panelBpp = static_cast<uint32_t>(atoi(tokens[1]));
            } else  if (!strncmp(tokens[0], "pattern", strlen("pattern"))) {
                *patternType = static_cast<uint32_t>(atoi(tokens[1]));
            }
        }
    }

    fclose(configFile);

    return 0;
}

DriverType getDriverType() {
    const char *fb_caps = "/sys/devices/virtual/graphics/fb0/mdp/caps";
    // 0 - File exists
    return access(fb_caps, F_OK) ? DriverType::DRM : DriverType::FB;
}

}; //namespace qdutils
