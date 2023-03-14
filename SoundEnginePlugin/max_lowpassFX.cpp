/*******************************************************************************
The content of this file includes portions of the AUDIOKINETIC Wwise Technology
released in source code form as part of the SDK installer package.

Commercial License Usage

Licensees holding valid commercial licenses to the AUDIOKINETIC Wwise Technology
may use this file in accordance with the end user license agreement provided
with the software or, alternatively, in accordance with the terms contained in a
written agreement between you and Audiokinetic Inc.

Apache License Usage

Alternatively, this file may be used under the Apache License, Version 2.0 (the
"Apache License"); you may not use this file except in compliance with the
Apache License. You may obtain a copy of the Apache License at
http://www.apache.org/licenses/LICENSE-2.0.

Unless required by applicable law or agreed to in writing, software distributed
under the Apache License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES
OR CONDITIONS OF ANY KIND, either express or implied. See the Apache License for
the specific language governing permissions and limitations under the License.

  Copyright (c) 2021 Audiokinetic Inc.
*******************************************************************************/

#include "max_lowpassFX.h"
#include "../max_lowpassConfig.h"

#include <AK/AkWwiseSDKVersion.h>


void UpdateBufferState(
    AkAudioBuffer* in_pBuffer,
    AkUInt16 in_uFramesConsumed,
    AkAudioBuffer* out_pBuffer,
    AkUInt16 in_uFramesProduced)
{
    in_pBuffer->uValidFrames -= in_uFramesConsumed;
    out_pBuffer->uValidFrames += in_uFramesProduced;
    if (in_pBuffer->eState == AK_NoMoreData && in_pBuffer->uValidFrames == 0)
    {
        out_pBuffer->eState = AK_NoMoreData;
    }
    else if (out_pBuffer->uValidFrames == out_pBuffer->MaxFrames())
    {
        out_pBuffer->eState = AK_DataReady;
    }
    else
    {
        out_pBuffer->eState = AK_DataNeeded;
    }
}


void TransferBuffer(
    AkAudioBuffer* in_pBuffer,
    AkAudioBuffer* out_pBuffer)
{
    auto const uSize = in_pBuffer->MaxFrames() * in_pBuffer->NumChannels() * sizeof(AkSampleType);

    const auto pReader = in_pBuffer->GetChannel(0);
    const auto pWriter = out_pBuffer->GetChannel(0);
    AKPLATFORM::AkMemCpy(pWriter, pReader, uSize);

    UpdateBufferState(in_pBuffer,
                      in_pBuffer->uValidFrames,
                      out_pBuffer,
                      in_pBuffer->uValidFrames);
}


AK::IAkPlugin* Createmax_lowpassFX(AK::IAkPluginMemAlloc* in_pAllocator)
{
    return AK_PLUGIN_NEW(in_pAllocator, max_lowpassFX());
}

AK::IAkPluginParam* Createmax_lowpassFXParams(AK::IAkPluginMemAlloc* in_pAllocator)
{
    return AK_PLUGIN_NEW(in_pAllocator, max_lowpassFXParams());
}

AK_IMPLEMENT_PLUGIN_FACTORY(max_lowpassFX, AkPluginTypeEffect, max_lowpassConfig::CompanyID, max_lowpassConfig::PluginID)

max_lowpassFX::max_lowpassFX()
    : m_pParams(nullptr)
    , m_pAllocator(nullptr)
    , m_pContext(nullptr)
    , m_pCore(new RNBO::CoreObject())
{
}

max_lowpassFX::~max_lowpassFX()
{
}

AKRESULT max_lowpassFX::Init(AK::IAkPluginMemAlloc* in_pAllocator, AK::IAkEffectPluginContext* in_pContext, AK::IAkPluginParam* in_pParams, AkAudioFormat& in_rFormat)
{
    m_pParams = (max_lowpassFXParams*)in_pParams;
    m_pAllocator = in_pAllocator;
    m_pContext = in_pContext;
    
    m_pCore->prepareToProcess(in_rFormat.uSampleRate, in_pContext->GlobalContext()->GetMaxBufferLength());
    m_pCore->setParameterValue(0, 500);

    return AK_Success;
}

AKRESULT max_lowpassFX::Term(AK::IAkPluginMemAlloc* in_pAllocator)
{
    delete m_pCore;
    AK_PLUGIN_DELETE(in_pAllocator, this);
    return AK_Success;
}

AKRESULT max_lowpassFX::Reset()
{
    return AK_Success;
}

AKRESULT max_lowpassFX::GetPluginInfo(AkPluginInfo& out_rPluginInfo)
{
    out_rPluginInfo.eType = AkPluginTypeEffect;
    out_rPluginInfo.bIsInPlace = false;
	out_rPluginInfo.bCanProcessObjects = false;
    out_rPluginInfo.uBuildVersion = AK_WWISESDK_VERSION_COMBINED;
    return AK_Success;
}

void max_lowpassFX::Execute(AkAudioBuffer* in_pBuffer, AkUInt32 in_ulnOffset, AkAudioBuffer* out_pBuffer)
{
    auto const uNumChannels = in_pBuffer->NumChannels();

    // Support stereo channel only
    if (uNumChannels != 2)
    {
        TransferBuffer(in_pBuffer, out_pBuffer);
        return;
    }
    
    UpdateParameters();
    AkSampleType* inputs[] = {in_pBuffer->GetChannel(0) + in_ulnOffset, in_pBuffer->GetChannel(1) + in_ulnOffset};
    AkSampleType* outputs[] = {out_pBuffer->GetChannel(0), out_pBuffer->GetChannel(1)};
    auto const uSampleFrames = in_pBuffer->uValidFrames;
    m_pCore->process(inputs, uNumChannels, outputs, uNumChannels, uSampleFrames);

    UpdateBufferState(in_pBuffer, uSampleFrames, out_pBuffer, uSampleFrames);
}

AKRESULT max_lowpassFX::TimeSkip(AkUInt32 &io_uFrames)
{
    return AK_DataReady;
}

void max_lowpassFX::UpdateParameters()
{
    auto changeHandler = m_pParams->m_paramChangeHandler;
    if (changeHandler.HasChanged(PARAM_FREQ_ID))
    {
        m_pCore->setParameterValue(PARAM_FREQ_ID, m_pParams->RTPC.freq);
        changeHandler.ResetParamChange(PARAM_FREQ_ID);
    }
}
