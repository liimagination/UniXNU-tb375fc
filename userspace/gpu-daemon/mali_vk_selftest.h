/* Standalone Mali-G615 Vulkan 1.1 hardware diagnostic self-test:
 *   Test A: Solid Green Triangle (no VB, no texture) over Red Clear -> 8x8 tile grid
 *   Test B: Sampled 64x64 4-Quadrant RGBA Texture (R/G/B/Y) uploaded via vkCmdCopyBufferToImage
 */
#pragma once

#define VK_NO_PROTOTYPES 1
#include <vulkan/vulkan.h>
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libkern/OSCacheControl.h>

#include "selftest-spv/tri_vert.h"
#include "selftest-spv/tri_frag.h"
#include "selftest-spv/tex_vert.h"
#include "selftest-spv/tex_frag.h"
#include "selftest-spv/comp.h"

typedef void (*MaliLogFn)(const char *fmt, ...);

static uint32_t mali_st_find_mem(const VkPhysicalDeviceMemoryProperties *mp,
                                 uint32_t typeBits, VkMemoryPropertyFlags req) {
    for (uint32_t i = 0; i < mp->memoryTypeCount; ++i) {
        if ((typeBits & (1u << i)) && (mp->memoryTypes[i].propertyFlags & req) == req)
            return i;
    }
    return UINT32_MAX;
}

static void run_mali_vk_selftest(const char *icd_path, MaliLogFn log_fn) {
    if (!icd_path || !log_fn) return;
    void *dl = dlopen(icd_path, RTLD_NOW | RTLD_LOCAL);
    if (!dl) {
        log_fn("[gpud] VK-SELFTEST: dlopen(%s) fail: %s\n", icd_path, dlerror());
        return;
    }
    PFN_vkGetInstanceProcAddr gipa =
        (PFN_vkGetInstanceProcAddr)dlsym(dl, "vk_icdGetInstanceProcAddr");
    if (!gipa) gipa = (PFN_vkGetInstanceProcAddr)dlsym(dl, "vkGetInstanceProcAddr");
    if (!gipa) {
        log_fn("[gpud] VK-SELFTEST: no vk_icdGetInstanceProcAddr\n");
        dlclose(dl);
        return;
    }

    auto vkCreateInstance = (PFN_vkCreateInstance)gipa(VK_NULL_HANDLE, "vkCreateInstance");
    if (!vkCreateInstance) { log_fn("[gpud] VK-SELFTEST: no vkCreateInstance\n"); dlclose(dl); return; }

    VkApplicationInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    ai.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo ici{};
    ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ici.pApplicationInfo = &ai;

    VkInstance inst = VK_NULL_HANDLE;
    VkResult vr = vkCreateInstance(&ici, nullptr, &inst);
    if (vr != VK_SUCCESS) {
        log_fn("[gpud] VK-SELFTEST: vkCreateInstance=%d\n", (int)vr);
        dlclose(dl);
        return;
    }

#define LOAD_I(fn) auto fn = (PFN_##fn)gipa(inst, #fn)
    LOAD_I(vkDestroyInstance);
    LOAD_I(vkEnumeratePhysicalDevices);
    LOAD_I(vkGetPhysicalDeviceProperties);
    LOAD_I(vkGetPhysicalDeviceMemoryProperties);
    LOAD_I(vkCreateDevice);
    LOAD_I(vkGetDeviceProcAddr);
#undef LOAD_I

    uint32_t pdCnt = 1;
    VkPhysicalDevice pd = VK_NULL_HANDLE;
    vr = vkEnumeratePhysicalDevices(inst, &pdCnt, &pd);
    if ((vr != VK_SUCCESS && vr != VK_INCOMPLETE) || pdCnt == 0) {
        log_fn("[gpud] VK-SELFTEST: no physical device (%d)\n", (int)vr);
        vkDestroyInstance(inst, nullptr);
        dlclose(dl);
        return;
    }
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(pd, &props);
    VkPhysicalDeviceMemoryProperties mp{};
    vkGetPhysicalDeviceMemoryProperties(pd, &mp);
    log_fn("[gpud] VK-SELFTEST: device='%s' api=0x%x memTypes=%u\n",
           props.deviceName, props.apiVersion, mp.memoryTypeCount);

    float qprio = 1.0f;
    VkDeviceQueueCreateInfo qci{};
    qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    qci.queueFamilyIndex = 0;
    qci.queueCount = 1;
    qci.pQueuePriorities = &qprio;
    VkDeviceCreateInfo dci{};
    dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;

    VkDevice dev = VK_NULL_HANDLE;
    vr = vkCreateDevice(pd, &dci, nullptr, &dev);
    if (vr != VK_SUCCESS) {
        log_fn("[gpud] VK-SELFTEST: vkCreateDevice=%d\n", (int)vr);
        vkDestroyInstance(inst, nullptr);
        dlclose(dl);
        return;
    }

#define LOAD_D(fn) auto fn = (PFN_##fn)vkGetDeviceProcAddr(dev, #fn)
    LOAD_D(vkDestroyDevice);
    LOAD_D(vkGetDeviceQueue);
    LOAD_D(vkCreateCommandPool);
    LOAD_D(vkDestroyCommandPool);
    LOAD_D(vkAllocateCommandBuffers);
    LOAD_D(vkBeginCommandBuffer);
    LOAD_D(vkEndCommandBuffer);
    LOAD_D(vkResetCommandBuffer);
    LOAD_D(vkCreateImage);
    LOAD_D(vkDestroyImage);
    LOAD_D(vkGetImageMemoryRequirements);
    LOAD_D(vkGetImageSubresourceLayout);
    LOAD_D(vkBindImageMemory);
    LOAD_D(vkCreateImageView);
    LOAD_D(vkDestroyImageView);
    LOAD_D(vkCreateBuffer);
    LOAD_D(vkDestroyBuffer);
    LOAD_D(vkGetBufferMemoryRequirements);
    LOAD_D(vkBindBufferMemory);
    LOAD_D(vkAllocateMemory);
    LOAD_D(vkFreeMemory);
    LOAD_D(vkMapMemory);
    LOAD_D(vkUnmapMemory);
    LOAD_D(vkCreateRenderPass);
    LOAD_D(vkDestroyRenderPass);
    LOAD_D(vkCreateFramebuffer);
    LOAD_D(vkDestroyFramebuffer);
    LOAD_D(vkCreateShaderModule);
    LOAD_D(vkDestroyShaderModule);
    LOAD_D(vkCreatePipelineLayout);
    LOAD_D(vkDestroyPipelineLayout);
    LOAD_D(vkCreateGraphicsPipelines);
    LOAD_D(vkCreateComputePipelines);
    LOAD_D(vkDestroyPipeline);
    LOAD_D(vkCmdDispatch);
    LOAD_D(vkCreateSampler);
    LOAD_D(vkDestroySampler);
    LOAD_D(vkCreateDescriptorSetLayout);
    LOAD_D(vkDestroyDescriptorSetLayout);
    LOAD_D(vkCreateDescriptorPool);
    LOAD_D(vkDestroyDescriptorPool);
    LOAD_D(vkAllocateDescriptorSets);
    LOAD_D(vkUpdateDescriptorSets);
    LOAD_D(vkCreateFence);
    LOAD_D(vkDestroyFence);
    LOAD_D(vkResetFences);
    LOAD_D(vkWaitForFences);
    LOAD_D(vkQueueSubmit);
    LOAD_D(vkCmdPipelineBarrier);
    LOAD_D(vkCmdCopyBufferToImage);
    LOAD_D(vkCmdBeginRenderPass);
    LOAD_D(vkCmdEndRenderPass);
    LOAD_D(vkCmdBindPipeline);
    LOAD_D(vkCmdBindDescriptorSets);
    LOAD_D(vkCmdDraw);
#undef LOAD_D

    VkQueue queue = VK_NULL_HANDLE;
    vkGetDeviceQueue(dev, 0, 0, &queue);

    const uint32_t W = 256, H = 256;
    VkImageCreateInfo rtCi{};
    rtCi.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    rtCi.imageType = VK_IMAGE_TYPE_2D;
    rtCi.format = VK_FORMAT_B8G8R8A8_UNORM;
    rtCi.extent = { W, H, 1 };
    rtCi.mipLevels = 1;
    rtCi.arrayLayers = 1;
    rtCi.samples = VK_SAMPLE_COUNT_1_BIT;
    rtCi.tiling = VK_IMAGE_TILING_LINEAR;
    rtCi.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    rtCi.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VkImage rtImg = VK_NULL_HANDLE;
    vr = vkCreateImage(dev, &rtCi, nullptr, &rtImg);
    if (vr != VK_SUCCESS) { log_fn("[gpud] VK-SELFTEST: rtImg=%d\n", (int)vr); goto done_dev; }

    {
        VkMemoryRequirements rtMr{};
        vkGetImageMemoryRequirements(dev, rtImg, &rtMr);
        uint32_t mt = mali_st_find_mem(&mp, rtMr.memoryTypeBits,
                                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        if (mt == UINT32_MAX) mt = mali_st_find_mem(&mp, rtMr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
        VkMemoryAllocateInfo mai{};
        mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        mai.allocationSize = rtMr.size;
        mai.memoryTypeIndex = mt;
        VkDeviceMemory rtMem = VK_NULL_HANDLE;
        vr = vkAllocateMemory(dev, &mai, nullptr, &rtMem);
        if (vr != VK_SUCCESS) { log_fn("[gpud] VK-SELFTEST: rtMem=%d\n", (int)vr); goto done_rt; }
        vkBindImageMemory(dev, rtImg, rtMem, 0);

        VkImageSubresource sub{};
        sub.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        VkSubresourceLayout lay{};
        vkGetImageSubresourceLayout(dev, rtImg, &sub, &lay);
        log_fn("[gpud] VK-SELFTEST: RT linear pitch=%llu off=%llu size=%llu\n",
               (unsigned long long)lay.rowPitch, (unsigned long long)lay.offset, (unsigned long long)rtMr.size);

        void *rtMap = nullptr;
        vkMapMemory(dev, rtMem, 0, rtMr.size, 0, &rtMap);

        VkImageViewCreateInfo ivci{};
        ivci.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        ivci.image = rtImg;
        ivci.viewType = VK_IMAGE_VIEW_TYPE_2D;
        ivci.format = VK_FORMAT_B8G8R8A8_UNORM;
        ivci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        VkImageView rtView = VK_NULL_HANDLE;
        vkCreateImageView(dev, &ivci, nullptr, &rtView);

        VkAttachmentDescription att{};
        att.format = VK_FORMAT_B8G8R8A8_UNORM;
        att.samples = VK_SAMPLE_COUNT_1_BIT;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att.finalLayout = VK_IMAGE_LAYOUT_GENERAL;
        VkAttachmentReference cref = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkSubpassDescription sp{};
        sp.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sp.colorAttachmentCount = 1;
        sp.pColorAttachments = &cref;
        VkRenderPassCreateInfo rpci{};
        rpci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpci.attachmentCount = 1;
        rpci.pAttachments = &att;
        rpci.subpassCount = 1;
        rpci.pSubpasses = &sp;
        VkRenderPass rp = VK_NULL_HANDLE;
        vkCreateRenderPass(dev, &rpci, nullptr, &rp);

        VkFramebufferCreateInfo fbci{};
        fbci.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbci.renderPass = rp;
        fbci.attachmentCount = 1;
        fbci.pAttachments = &rtView;
        fbci.width = W; fbci.height = H; fbci.layers = 1;
        VkFramebuffer fb = VK_NULL_HANDLE;
        vkCreateFramebuffer(dev, &fbci, nullptr, &fb);

        VkCommandPoolCreateInfo cpci{};
        cpci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        cpci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        VkCommandPool pool = VK_NULL_HANDLE;
        vkCreateCommandPool(dev, &cpci, nullptr, &pool);
        VkCommandBufferAllocateInfo cbai{};
        cbai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        cbai.commandPool = pool;
        cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cbai.commandBufferCount = 1;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        vkAllocateCommandBuffers(dev, &cbai, &cmd);

        VkFenceCreateInfo fci{};
        fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        VkFence fence = VK_NULL_HANDLE;
        vkCreateFence(dev, &fci, nullptr, &fence);

        auto make_pipe = [&](const uint32_t *vs, size_t vsz, const uint32_t *fs, size_t fsz,
                             VkPipelineLayout pl) -> VkPipeline {
            VkShaderModuleCreateInfo smci{};
            smci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            smci.codeSize = vsz; smci.pCode = vs;
            VkShaderModule vm = VK_NULL_HANDLE, fm = VK_NULL_HANDLE;
            vkCreateShaderModule(dev, &smci, nullptr, &vm);
            smci.codeSize = fsz; smci.pCode = fs;
            vkCreateShaderModule(dev, &smci, nullptr, &fm);

            VkPipelineShaderStageCreateInfo stages[2]{};
            stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
            stages[0].module = vm; stages[0].pName = "main";
            stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            stages[1].module = fm; stages[1].pName = "main";

            VkPipelineVertexInputStateCreateInfo vi{};
            vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
            VkPipelineInputAssemblyStateCreateInfo ia{};
            ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
            ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
            VkViewport vp = { 0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f };
            VkRect2D sc = { {0, 0}, {W, H} };
            VkPipelineViewportStateCreateInfo vps{};
            vps.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
            vps.viewportCount = 1; vps.pViewports = &vp;
            vps.scissorCount = 1; vps.pScissors = &sc;
            VkPipelineRasterizationStateCreateInfo rs{};
            rs.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
            rs.polygonMode = VK_POLYGON_MODE_FILL;
            rs.cullMode = VK_CULL_MODE_NONE;
            rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
            rs.lineWidth = 1.0f;
            VkPipelineMultisampleStateCreateInfo ms{};
            ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
            ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
            VkPipelineColorBlendAttachmentState cba{};
            cba.colorWriteMask = 0xF;
            VkPipelineColorBlendStateCreateInfo cb{};
            cb.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            cb.attachmentCount = 1; cb.pAttachments = &cba;

            VkGraphicsPipelineCreateInfo gpci{};
            gpci.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            gpci.stageCount = 2; gpci.pStages = stages;
            gpci.pVertexInputState = &vi;
            gpci.pInputAssemblyState = &ia;
            gpci.pViewportState = &vps;
            gpci.pRasterizationState = &rs;
            gpci.pMultisampleState = &ms;
            gpci.pColorBlendState = &cb;
            gpci.layout = pl;
            gpci.renderPass = rp;
            VkPipeline pipe = VK_NULL_HANDLE;
            VkResult pvr = vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &gpci, nullptr, &pipe);
            log_fn("[gpud] VK-SELFTEST: make_pipe vr=%d pipe=%p\n", (int)pvr, (void *)pipe);
            vkDestroyShaderModule(dev, vm, nullptr);
            vkDestroyShaderModule(dev, fm, nullptr);
            return pipe;
        };

        auto read_px = [&](uint32_t x, uint32_t y) -> uint32_t {
            const uint8_t *p = (const uint8_t *)rtMap + lay.offset + y * lay.rowPitch + x * 4;
            uint32_t v = 0; memcpy(&v, p, 4); return v;
        };

        /* ---- TEST A: Solid Green Triangle over Red Clear + VS/FS SSBO probe ---- */
        {
            VkBufferCreateInfo bci{};
            bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            bci.size = 256;
            bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VkBuffer dbgBuf = VK_NULL_HANDLE;
            vkCreateBuffer(dev, &bci, nullptr, &dbgBuf);
            VkMemoryRequirements bmr{};
            vkGetBufferMemoryRequirements(dev, dbgBuf, &bmr);
            VkMemoryAllocateInfo bmai{};
            bmai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            bmai.allocationSize = bmr.size;
            bmai.memoryTypeIndex = mali_st_find_mem(&mp, bmr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
            VkDeviceMemory dbgMem = VK_NULL_HANDLE;
            vkAllocateMemory(dev, &bmai, nullptr, &dbgMem);
            vkBindBufferMemory(dev, dbgBuf, dbgMem, 0);
            void *dMap = nullptr;
            vkMapMemory(dev, dbgMem, 0, bmr.size, 0, &dMap);
            memset(dMap, 0, bmr.size);
            sys_dcache_flush(dMap, bmr.size);

            VkDescriptorSetLayoutBinding bind{};
            bind.binding = 0;
            bind.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bind.descriptorCount = 1;
            bind.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
            VkDescriptorSetLayoutCreateInfo dslci{};
            dslci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            dslci.bindingCount = 1; dslci.pBindings = &bind;
            VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
            vkCreateDescriptorSetLayout(dev, &dslci, nullptr, &dsl);

            VkDescriptorPoolSize dps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1 };
            VkDescriptorPoolCreateInfo dpci{};
            dpci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            dpci.maxSets = 1; dpci.poolSizeCount = 1; dpci.pPoolSizes = &dps;
            VkDescriptorPool dp = VK_NULL_HANDLE;
            vkCreateDescriptorPool(dev, &dpci, nullptr, &dp);

            VkDescriptorSetAllocateInfo dsai{};
            dsai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            dsai.descriptorPool = dp; dsai.descriptorSetCount = 1; dsai.pSetLayouts = &dsl;
            VkDescriptorSet ds = VK_NULL_HANDLE;
            vkAllocateDescriptorSets(dev, &dsai, &ds);

            VkDescriptorBufferInfo dbi = { dbgBuf, 0, 256 };
            VkWriteDescriptorSet wds{};
            wds.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            wds.dstSet = ds; wds.dstBinding = 0; wds.descriptorCount = 1;
            wds.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            wds.pBufferInfo = &dbi;
            vkUpdateDescriptorSets(dev, 1, &wds, 0, nullptr);

            VkPipelineLayoutCreateInfo plci{};
            plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            plci.setLayoutCount = 1; plci.pSetLayouts = &dsl;
            VkPipelineLayout pl = VK_NULL_HANDLE;
            vkCreatePipelineLayout(dev, &plci, nullptr, &pl);
            VkPipeline pipe = make_pipe(spv_tri_vert, sizeof(spv_tri_vert),
                                        spv_tri_frag, sizeof(spv_tri_frag), pl);
            memset(rtMap, 0, rtMr.size);
            sys_dcache_flush(rtMap, rtMr.size);

            VkCommandBufferBeginInfo bi{};
            bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            vkBeginCommandBuffer(cmd, &bi);
            VkClearValue cv{};
            cv.color.float32[0] = 1.0f; cv.color.float32[1] = 0.0f;
            cv.color.float32[2] = 0.0f; cv.color.float32[3] = 1.0f;
            VkRenderPassBeginInfo rpbi{};
            rpbi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            rpbi.renderPass = rp; rpbi.framebuffer = fb;
            rpbi.renderArea = { {0, 0}, {W, H} };
            rpbi.clearValueCount = 1; rpbi.pClearValues = &cv;
            vkCmdBeginRenderPass(cmd, &rpbi, VK_SUBPASS_CONTENTS_INLINE);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pl, 0, 1, &ds, 0, nullptr);
            vkCmdDraw(cmd, 3, 1, 0, 0);
            vkCmdEndRenderPass(cmd);
            vkEndCommandBuffer(cmd);

            VkSubmitInfo si{};
            si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            si.commandBufferCount = 1; si.pCommandBuffers = &cmd;
            vkResetFences(dev, 1, &fence);
            vkQueueSubmit(queue, 1, &si, fence);
            vr = vkWaitForFences(dev, 1, &fence, VK_TRUE, 3000000000ULL);
            sys_dcache_flush(rtMap, rtMr.size);
            sys_dcache_flush(dMap, bmr.size);
            const uint32_t *hv = (const uint32_t *)dMap;

            uint32_t tl = read_px(16, 16), tr = read_px(240, 16);
            uint32_t bl = read_px(16, 240), br = read_px(240, 240);
            log_fn("[gpud] VK-SELFTEST-A (GreenTri/RedClr): wait=%d vsHit=0x%x/0x%x/0x%x fsHit=0x%x TL=0x%08x(want 0xff00ff00) BR=0x%08x(want 0xffff0000)\n",
                   (int)vr, hv[0], hv[1], hv[2], hv[3], tl, br);
            for (uint32_t gy = 0; gy < 8; ++gy) {
                char row[32]; int pos = 0;
                for (uint32_t gx = 0; gx < 8; ++gx) {
                    uint32_t c = read_px(gx * 32 + 16, gy * 32 + 16);
                    char ch = (c == 0xff00ff00u) ? 'G' : (c == 0xffff0000u) ? 'R' : (c == 0 ? '0' : '?');
                    row[pos++] = ch; row[pos++] = ' ';
                }
                row[pos] = 0;
                log_fn("[gpud] VK-SELFTEST-A grid[%u]: %s\n", gy, row);
            }
            vkDestroyPipeline(dev, pipe, nullptr);
            vkDestroyPipelineLayout(dev, pl, nullptr);
            vkDestroyDescriptorPool(dev, dp, nullptr);
            vkDestroyDescriptorSetLayout(dev, dsl, nullptr);
            vkUnmapMemory(dev, dbgMem);
            vkDestroyBuffer(dev, dbgBuf, nullptr);
            vkFreeMemory(dev, dbgMem, nullptr);
        }

        /* ---- TEST B: Sampled 64x64 4-Quadrant RGBA Texture (OPTIMAL) ---- */
        {
            const uint32_t TW = 64, TH = 64;
            VkBufferCreateInfo bci{};
            bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            bci.size = TW * TH * 4;
            bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            VkBuffer stagBuf = VK_NULL_HANDLE;
            vkCreateBuffer(dev, &bci, nullptr, &stagBuf);
            VkMemoryRequirements bmr{};
            vkGetBufferMemoryRequirements(dev, stagBuf, &bmr);
            VkMemoryAllocateInfo bmai{};
            bmai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            bmai.allocationSize = bmr.size;
            bmai.memoryTypeIndex = mali_st_find_mem(&mp, bmr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
            VkDeviceMemory stagMem = VK_NULL_HANDLE;
            vkAllocateMemory(dev, &bmai, nullptr, &stagMem);
            vkBindBufferMemory(dev, stagBuf, stagMem, 0);
            void *sMap = nullptr;
            vkMapMemory(dev, stagMem, 0, bmr.size, 0, &sMap);
            uint32_t *spix = (uint32_t *)sMap;
            /* VK_FORMAT_R8G8B8A8_UNORM: 0xAABBGGRR in LE */
            for (uint32_t y = 0; y < TH; ++y) {
                for (uint32_t x = 0; x < TW; ++x) {
                    bool right = (x >= TW / 2), bot = (y >= TH / 2);
                    uint32_t rgba = (!bot && !right) ? 0xff0000ffu : /* TL: Red */
                                    (!bot &&  right) ? 0xff00ff00u : /* TR: Green */
                                    ( bot && !right) ? 0xffff0000u : /* BL: Blue */
                                                       0xff00ffffu;  /* BR: Yellow */
                    spix[y * TW + x] = rgba;
                }
            }
            sys_dcache_flush(sMap, bmr.size);
            vkUnmapMemory(dev, stagMem);

            VkImageCreateInfo tci{};
            tci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            tci.imageType = VK_IMAGE_TYPE_2D;
            tci.format = VK_FORMAT_R8G8B8A8_UNORM;
            tci.extent = { TW, TH, 1 };
            tci.mipLevels = 1; tci.arrayLayers = 1;
            tci.samples = VK_SAMPLE_COUNT_1_BIT;
            tci.tiling = VK_IMAGE_TILING_OPTIMAL;
            tci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            VkImage texImg = VK_NULL_HANDLE;
            vkCreateImage(dev, &tci, nullptr, &texImg);
            VkMemoryRequirements tmr{};
            vkGetImageMemoryRequirements(dev, texImg, &tmr);
            VkMemoryAllocateInfo tmai{};
            tmai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            tmai.allocationSize = tmr.size;
            tmai.memoryTypeIndex = mali_st_find_mem(&mp, tmr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            VkDeviceMemory texMem = VK_NULL_HANDLE;
            vkAllocateMemory(dev, &tmai, nullptr, &texMem);
            vkBindImageMemory(dev, texImg, texMem, 0);

            VkImageViewCreateInfo tvci{};
            tvci.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            tvci.image = texImg; tvci.viewType = VK_IMAGE_VIEW_TYPE_2D;
            tvci.format = VK_FORMAT_R8G8B8A8_UNORM;
            tvci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            VkImageView texView = VK_NULL_HANDLE;
            vkCreateImageView(dev, &tvci, nullptr, &texView);

            VkSamplerCreateInfo sci{};
            sci.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
            sci.magFilter = VK_FILTER_NEAREST; sci.minFilter = VK_FILTER_NEAREST;
            sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            VkSampler samp = VK_NULL_HANDLE;
            vkCreateSampler(dev, &sci, nullptr, &samp);

            VkDescriptorSetLayoutBinding bind{};
            bind.binding = 0;
            bind.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            bind.descriptorCount = 1;
            bind.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
            VkDescriptorSetLayoutCreateInfo dslci{};
            dslci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            dslci.bindingCount = 1; dslci.pBindings = &bind;
            VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
            vkCreateDescriptorSetLayout(dev, &dslci, nullptr, &dsl);

            VkDescriptorPoolSize dps = { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1 };
            VkDescriptorPoolCreateInfo dpci{};
            dpci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            dpci.maxSets = 1; dpci.poolSizeCount = 1; dpci.pPoolSizes = &dps;
            VkDescriptorPool dp = VK_NULL_HANDLE;
            vkCreateDescriptorPool(dev, &dpci, nullptr, &dp);

            VkDescriptorSetAllocateInfo dsai{};
            dsai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            dsai.descriptorPool = dp; dsai.descriptorSetCount = 1; dsai.pSetLayouts = &dsl;
            VkDescriptorSet ds = VK_NULL_HANDLE;
            vkAllocateDescriptorSets(dev, &dsai, &ds);

            VkDescriptorImageInfo dii = { samp, texView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            VkWriteDescriptorSet wds{};
            wds.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            wds.dstSet = ds; wds.dstBinding = 0; wds.descriptorCount = 1;
            wds.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            wds.pImageInfo = &dii;
            vkUpdateDescriptorSets(dev, 1, &wds, 0, nullptr);

            VkPipelineLayoutCreateInfo plci{};
            plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            plci.setLayoutCount = 1; plci.pSetLayouts = &dsl;
            VkPipelineLayout pl = VK_NULL_HANDLE;
            vkCreatePipelineLayout(dev, &plci, nullptr, &pl);
            VkPipeline pipe = make_pipe(spv_tex_vert, sizeof(spv_tex_vert),
                                        spv_tex_frag, sizeof(spv_tex_frag), pl);

            memset(rtMap, 0, rtMr.size);
            sys_dcache_flush(rtMap, rtMr.size);
            vkResetCommandBuffer(cmd, 0);
            VkCommandBufferBeginInfo bi{};
            bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            vkBeginCommandBuffer(cmd, &bi);

            VkImageMemoryBarrier b1{};
            b1.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            b1.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            b1.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            b1.image = texImg;
            b1.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            b1.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 0, 0, nullptr, 0, nullptr, 1, &b1);

            VkBufferImageCopy reg{};
            reg.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
            reg.imageExtent = { TW, TH, 1 };
            vkCmdCopyBufferToImage(cmd, stagBuf, texImg, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &reg);

            VkImageMemoryBarrier b2{};
            b2.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            b2.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            b2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            b2.image = texImg;
            b2.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            b2.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            b2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                 0, 0, nullptr, 0, nullptr, 1, &b2);

            VkClearValue cv{};
            VkRenderPassBeginInfo rpbi{};
            rpbi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            rpbi.renderPass = rp; rpbi.framebuffer = fb;
            rpbi.renderArea = { {0, 0}, {W, H} };
            rpbi.clearValueCount = 1; rpbi.pClearValues = &cv;
            vkCmdBeginRenderPass(cmd, &rpbi, VK_SUBPASS_CONTENTS_INLINE);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pl, 0, 1, &ds, 0, nullptr);
            vkCmdDraw(cmd, 3, 1, 0, 0);
            vkCmdEndRenderPass(cmd);
            vkEndCommandBuffer(cmd);

            VkSubmitInfo si{};
            si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            si.commandBufferCount = 1; si.pCommandBuffers = &cmd;
            vkResetFences(dev, 1, &fence);
            vkQueueSubmit(queue, 1, &si, fence);
            vr = vkWaitForFences(dev, 1, &fence, VK_TRUE, 3000000000ULL);
            sys_dcache_flush(rtMap, rtMr.size);

            uint32_t tl = read_px(64, 64), tr = read_px(192, 64);
            uint32_t bl = read_px(64, 192), br = read_px(192, 192);
            bool ok = (tl == 0xffff0000u && tr == 0xff00ff00u && bl == 0xff0000ffu && br == 0xffffff00u);
            log_fn("[gpud] VK-SELFTEST-B (4-Color TexQuad): wait=%d TL=0x%08x(R) TR=0x%08x(G) BL=0x%08x(B) BR=0x%08x(Y) -> %s\n",
                   (int)vr, tl, tr, bl, br, ok ? "PASS!" : "FAIL!");

            vkDestroyPipeline(dev, pipe, nullptr);
            vkDestroyPipelineLayout(dev, pl, nullptr);
            vkDestroyDescriptorPool(dev, dp, nullptr);
            vkDestroyDescriptorSetLayout(dev, dsl, nullptr);
            vkDestroySampler(dev, samp, nullptr);
            vkDestroyImageView(dev, texView, nullptr);
            vkDestroyImage(dev, texImg, nullptr);
            vkFreeMemory(dev, texMem, nullptr);
            vkDestroyBuffer(dev, stagBuf, nullptr);
            vkFreeMemory(dev, stagMem, nullptr);
        }

        /* ---- TEST C: Compute Shader writing 0x12345678 / 0xdeadbeef ---- */
        {
            VkBufferCreateInfo bci{};
            bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            bci.size = 256;
            bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VkBuffer ssbo = VK_NULL_HANDLE;
            vkCreateBuffer(dev, &bci, nullptr, &ssbo);
            VkMemoryRequirements bmr{};
            vkGetBufferMemoryRequirements(dev, ssbo, &bmr);
            VkMemoryAllocateInfo bmai{};
            bmai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            bmai.allocationSize = bmr.size;
            bmai.memoryTypeIndex = mali_st_find_mem(&mp, bmr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
            VkDeviceMemory smem = VK_NULL_HANDLE;
            vkAllocateMemory(dev, &bmai, nullptr, &smem);
            vkBindBufferMemory(dev, ssbo, smem, 0);
            void *sMap = nullptr;
            vkMapMemory(dev, smem, 0, bmr.size, 0, &sMap);
            memset(sMap, 0, bmr.size);
            sys_dcache_flush(sMap, bmr.size);

            VkDescriptorSetLayoutBinding bind{};
            bind.binding = 0;
            bind.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bind.descriptorCount = 1;
            bind.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
            VkDescriptorSetLayoutCreateInfo dslci{};
            dslci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            dslci.bindingCount = 1; dslci.pBindings = &bind;
            VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
            vkCreateDescriptorSetLayout(dev, &dslci, nullptr, &dsl);

            VkDescriptorPoolSize dps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1 };
            VkDescriptorPoolCreateInfo dpci{};
            dpci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            dpci.maxSets = 1; dpci.poolSizeCount = 1; dpci.pPoolSizes = &dps;
            VkDescriptorPool dp = VK_NULL_HANDLE;
            vkCreateDescriptorPool(dev, &dpci, nullptr, &dp);

            VkDescriptorSetAllocateInfo dsai{};
            dsai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            dsai.descriptorPool = dp; dsai.descriptorSetCount = 1; dsai.pSetLayouts = &dsl;
            VkDescriptorSet ds = VK_NULL_HANDLE;
            vkAllocateDescriptorSets(dev, &dsai, &ds);

            VkDescriptorBufferInfo dbi = { ssbo, 0, 256 };
            VkWriteDescriptorSet wds{};
            wds.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            wds.dstSet = ds; wds.dstBinding = 0; wds.descriptorCount = 1;
            wds.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            wds.pBufferInfo = &dbi;
            vkUpdateDescriptorSets(dev, 1, &wds, 0, nullptr);

            VkPipelineLayoutCreateInfo plci{};
            plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            plci.setLayoutCount = 1; plci.pSetLayouts = &dsl;
            VkPipelineLayout pl = VK_NULL_HANDLE;
            vkCreatePipelineLayout(dev, &plci, nullptr, &pl);

            VkShaderModuleCreateInfo smci{};
            smci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            smci.codeSize = sizeof(spv_comp); smci.pCode = spv_comp;
            VkShaderModule cm = VK_NULL_HANDLE;
            vkCreateShaderModule(dev, &smci, nullptr, &cm);

            VkComputePipelineCreateInfo cpci2{};
            cpci2.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
            cpci2.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            cpci2.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
            cpci2.stage.module = cm; cpci2.stage.pName = "main";
            cpci2.layout = pl;
            VkPipeline cpipe = VK_NULL_HANDLE;
            VkResult cvr = vkCreateComputePipelines(dev, VK_NULL_HANDLE, 1, &cpci2, nullptr, &cpipe);
            vkDestroyShaderModule(dev, cm, nullptr);

            vkResetCommandBuffer(cmd, 0);
            VkCommandBufferBeginInfo bi{};
            bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            vkBeginCommandBuffer(cmd, &bi);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cpipe);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pl, 0, 1, &ds, 0, nullptr);
            vkCmdDispatch(cmd, 1, 1, 1);
            vkEndCommandBuffer(cmd);

            VkSubmitInfo si{};
            si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            si.commandBufferCount = 1; si.pCommandBuffers = &cmd;
            vkResetFences(dev, 1, &fence);
            vkQueueSubmit(queue, 1, &si, fence);
            vr = vkWaitForFences(dev, 1, &fence, VK_TRUE, 3000000000ULL);
            sys_dcache_flush(sMap, bmr.size);
            const uint32_t *outv = (const uint32_t *)sMap;
            log_fn("[gpud] VK-SELFTEST-C (Compute SSBO): pipeVr=%d wait=%d v0=0x%08x(want 0x12345678) v1=0x%08x(want 0xdeadbeef) -> %s\n",
                   (int)cvr, (int)vr, outv[0], outv[1],
                   (outv[0] == 0x12345678u && outv[1] == 0xdeadbeefu) ? "PASS!" : "FAIL!");

            vkDestroyPipeline(dev, cpipe, nullptr);
            vkDestroyPipelineLayout(dev, pl, nullptr);
            vkDestroyDescriptorPool(dev, dp, nullptr);
            vkDestroyDescriptorSetLayout(dev, dsl, nullptr);
            vkUnmapMemory(dev, smem);
            vkDestroyBuffer(dev, ssbo, nullptr);
            vkFreeMemory(dev, smem, nullptr);
        }

        vkDestroyFence(dev, fence, nullptr);
        vkDestroyCommandPool(dev, pool, nullptr);
        vkDestroyFramebuffer(dev, fb, nullptr);
        vkDestroyRenderPass(dev, rp, nullptr);
        vkDestroyImageView(dev, rtView, nullptr);
        vkUnmapMemory(dev, rtMem);
        vkFreeMemory(dev, rtMem, nullptr);
    }
done_rt:
    vkDestroyImage(dev, rtImg, nullptr);
done_dev:
    vkDestroyDevice(dev, nullptr);
    vkDestroyInstance(inst, nullptr);
    dlclose(dl);
}
