/*******************************************************************************************************************
Copyright (c) 2023 Cycling '74

The code that Max generates automatically and that end users are capable of
exporting and using, and any associated documentation files (the “Software”)
is a work of authorship for which Cycling '74 is the author and owner for
copyright purposes.

This Software is dual-licensed either under the terms of the Cycling '74
License for Max-Generated Code for Export, or alternatively under the terms
of the General Public License (GPL) Version 3. You may use the Software
according to either of these licenses as it is most appropriate for your
project on a case-by-case basis (proprietary or not).

A) Cycling '74 License for Max-Generated Code for Export

A license is hereby granted, free of charge, to any person obtaining a copy
of the Software (“Licensee”) to use, copy, modify, merge, publish, and
distribute copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The Software is licensed to Licensee for all uses that do not include the sale,
sublicensing, or commercial distribution of software that incorporates this
source code. This means that the Licensee is free to use this software for
educational, research, and prototyping purposes, to create musical or other
creative works with software that incorporates this source code, or any other
use that does not constitute selling software that makes use of this source
code. Commercial distribution also includes the packaging of free software with
other paid software, hardware, or software-provided commercial services.

For entities with UNDER $200k in annual revenue or funding, a license is hereby
granted, free of charge, for the sale, sublicensing, or commercial distribution
of software that incorporates this source code, for as long as the entity's
annual revenue remains below $200k annual revenue or funding.

For entities with OVER $200k in annual revenue or funding interested in the
sale, sublicensing, or commercial distribution of software that incorporates
this source code, please send inquiries to licensing@cycling74.com.

The above copyright notice and this license shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Please see
https://support.cycling74.com/hc/en-us/articles/10730637742483-RNBO-Export-Licensing-FAQ
for additional information

B) General Public License Version 3 (GPLv3)
Details of the GPLv3 license can be found at: https://www.gnu.org/licenses/gpl-3.0.html
*******************************************************************************************************************/

#include "RNBO_Common.h"
#include "RNBO_AudioSignal.h"

namespace RNBO {


#define floor(x) ((long)(x))

#if defined(__GNUC__) || defined(__clang__)
    #define RNBO_RESTRICT __restrict__
#elif defined(_MSC_VER)
    #define RNBO_RESTRICT __restrict
#endif

#define FIXEDSIZEARRAYINIT(...) { }

class rnbomatic : public PatcherInterfaceImpl {
public:

rnbomatic()
{
}

~rnbomatic()
{
}

rnbomatic* getTopLevelPatcher() {
    return this;
}

void cancelClockEvents()
{
    getEngine()->flushClockEvents(this, 760652352, false);
}

template <typename T> void listquicksort(T& arr, T& sortindices, Int l, Int h, bool ascending) {
    if (l < h) {
        Int p = (Int)(this->listpartition(arr, sortindices, l, h, ascending));
        this->listquicksort(arr, sortindices, l, p - 1, ascending);
        this->listquicksort(arr, sortindices, p + 1, h, ascending);
    }
}

template <typename T> Int listpartition(T& arr, T& sortindices, Int l, Int h, bool ascending) {
    number x = arr[(Index)h];
    Int i = (Int)(l - 1);

    for (Int j = (Int)(l); j <= h - 1; j++) {
        bool asc = (bool)((bool)(ascending) && arr[(Index)j] <= x);
        bool desc = (bool)((bool)(!(bool)(ascending)) && arr[(Index)j] >= x);

        if ((bool)(asc) || (bool)(desc)) {
            i++;
            this->listswapelements(arr, i, j);
            this->listswapelements(sortindices, i, j);
        }
    }

    i++;
    this->listswapelements(arr, i, h);
    this->listswapelements(sortindices, i, h);
    return i;
}

template <typename T> void listswapelements(T& arr, Int a, Int b) {
    auto tmp = arr[(Index)a];
    arr[(Index)a] = arr[(Index)b];
    arr[(Index)b] = tmp;
}

SampleIndex currentsampletime() {
    return this->audioProcessSampleCount + this->sampleOffsetIntoNextAudioBuffer;
}

number mstosamps(MillisecondTime ms) {
    return ms * this->sr * 0.001;
}

inline number safesqrt(number num) {
    return (num > 0.0 ? rnbo_sqrt(num) : 0.0);
}

Index vectorsize() {
    return this->vs;
}

MillisecondTime currenttime() {
    return this->_currentTime;
}

number tempo() {
    return this->getTopLevelPatcher()->globaltransport_getTempo();
}

number mstobeats(number ms) {
    return ms * this->tempo() * 0.008 / (number)480;
}

MillisecondTime sampstoms(number samps) {
    return samps * 1000 / this->sr;
}

Index getNumMidiInputPorts() const {
    return 0;
}

void processMidiEvent(MillisecondTime , int , ConstByteArray , Index ) {}

Index getNumMidiOutputPorts() const {
    return 0;
}

void process(
    const SampleValue * const* inputs,
    Index numInputs,
    SampleValue * const* outputs,
    Index numOutputs,
    Index n
) {
    this->vs = n;
    this->updateTime(this->getEngine()->getCurrentTime());
    SampleValue * out1 = (numOutputs >= 1 && outputs[0] ? outputs[0] : this->dummyBuffer);
    SampleValue * out2 = (numOutputs >= 2 && outputs[1] ? outputs[1] : this->dummyBuffer);
    const SampleValue * in1 = (numInputs >= 1 && inputs[0] ? inputs[0] : this->zeroBuffer);
    const SampleValue * in2 = (numInputs >= 2 && inputs[1] ? inputs[1] : this->zeroBuffer);
    this->line_01_perform(this->signals[0], n);

    this->filtercoeff_01_perform(
        this->signals[0],
        this->filtercoeff_01_gain,
        this->filtercoeff_01_q,
        this->signals[1],
        this->signals[2],
        this->signals[3],
        this->signals[4],
        this->signals[5],
        n
    );

    this->biquad_tilde_02_perform(
        in2,
        this->signals[1],
        this->signals[2],
        this->signals[3],
        this->signals[4],
        this->signals[5],
        out2,
        n
    );

    this->biquad_tilde_01_perform(
        in1,
        this->signals[1],
        this->signals[2],
        this->signals[3],
        this->signals[4],
        this->signals[5],
        out1,
        n
    );

    this->stackprotect_perform(n);
    this->globaltransport_advance();
    this->audioProcessSampleCount += this->vs;
}

void prepareToProcess(number sampleRate, Index maxBlockSize, bool force) {
    if (this->maxvs < maxBlockSize || !this->didAllocateSignals) {
        Index i;

        for (i = 0; i < 6; i++) {
            this->signals[i] = resizeSignal(this->signals[i], this->maxvs, maxBlockSize);
        }

        this->globaltransport_tempo = resizeSignal(this->globaltransport_tempo, this->maxvs, maxBlockSize);
        this->globaltransport_state = resizeSignal(this->globaltransport_state, this->maxvs, maxBlockSize);
        this->zeroBuffer = resizeSignal(this->zeroBuffer, this->maxvs, maxBlockSize);
        this->dummyBuffer = resizeSignal(this->dummyBuffer, this->maxvs, maxBlockSize);
        this->didAllocateSignals = true;
    }

    const bool sampleRateChanged = sampleRate != this->sr;
    const bool maxvsChanged = maxBlockSize != this->maxvs;
    const bool forceDSPSetup = sampleRateChanged || maxvsChanged || force;

    if (sampleRateChanged || maxvsChanged) {
        this->vs = maxBlockSize;
        this->maxvs = maxBlockSize;
        this->sr = sampleRate;
        this->invsr = 1 / sampleRate;
    }

    this->filtercoeff_01_dspsetup(forceDSPSetup);
    this->biquad_tilde_02_dspsetup(forceDSPSetup);
    this->biquad_tilde_01_dspsetup(forceDSPSetup);
    this->globaltransport_dspsetup(forceDSPSetup);

    if (sampleRateChanged)
        this->onSampleRateChanged(sampleRate);
}

void setProbingTarget(MessageTag id) {
    switch (id) {
    default:
        {
        this->setProbingIndex(-1);
        break;
        }
    }
}

void setProbingIndex(ProbingIndex ) {}

Index getProbingChannels(MessageTag outletId) const {
    RNBO_UNUSED(outletId);
    return 0;
}

DataRef* getDataRef(DataRefIndex index)  {
    switch (index) {
    default:
        {
        return nullptr;
        }
    }
}

DataRefIndex getNumDataRefs() const {
    return 0;
}

void fillDataRef(DataRefIndex , DataRef& ) {}

void processDataViewUpdate(DataRefIndex , MillisecondTime ) {}

void initialize() {
    this->assign_defaults();
    this->setState();
    this->initializeObjects();
    this->allocateDataRefs();
    this->startup();
}

Index getIsMuted()  {
    return this->isMuted;
}

void setIsMuted(Index v)  {
    this->isMuted = v;
}

Index getPatcherSerial() const {
    return 0;
}

void getState(PatcherStateInterface& ) {}

void setState() {}

void getPreset(PatcherStateInterface& preset) {
    preset["__presetid"] = "rnbo";
    this->param_01_getPresetValue(getSubState(preset, "freq"));
}

void setPreset(MillisecondTime time, PatcherStateInterface& preset) {
    this->updateTime(time);
    this->param_01_setPresetValue(getSubState(preset, "freq"));
}

void processTempoEvent(MillisecondTime time, Tempo tempo) {
    this->updateTime(time);

    if (this->globaltransport_setTempo(tempo, false))
        {}
}

void processTransportEvent(MillisecondTime time, TransportState state) {
    this->updateTime(time);

    if (this->globaltransport_setState(state, false))
        {}
}

void processBeatTimeEvent(MillisecondTime time, BeatTime beattime) {
    this->updateTime(time);

    if (this->globaltransport_setBeatTime(beattime, false))
        {}
}

void onSampleRateChanged(double ) {}

void processTimeSignatureEvent(MillisecondTime time, int numerator, int denominator) {
    this->updateTime(time);

    if (this->globaltransport_setTimeSignature(numerator, denominator, false))
        {}
}

void setParameterValue(ParameterIndex index, ParameterValue v, MillisecondTime time) {
    this->updateTime(time);

    switch (index) {
    case 0:
        {
        this->param_01_value_set(v);
        break;
        }
    }
}

void processParameterEvent(ParameterIndex index, ParameterValue value, MillisecondTime time) {
    this->setParameterValue(index, value, time);
}

void processNormalizedParameterEvent(ParameterIndex index, ParameterValue value, MillisecondTime time) {
    this->setParameterValueNormalized(index, value, time);
}

ParameterValue getParameterValue(ParameterIndex index)  {
    switch (index) {
    case 0:
        {
        return this->param_01_value;
        }
    default:
        {
        return 0;
        }
    }
}

ParameterIndex getNumSignalInParameters() const {
    return 0;
}

ParameterIndex getNumSignalOutParameters() const {
    return 0;
}

ParameterIndex getNumParameters() const {
    return 1;
}

ConstCharPointer getParameterName(ParameterIndex index) const {
    switch (index) {
    case 0:
        {
        return "freq";
        }
    default:
        {
        return "bogus";
        }
    }
}

ConstCharPointer getParameterId(ParameterIndex index) const {
    switch (index) {
    case 0:
        {
        return "freq";
        }
    default:
        {
        return "bogus";
        }
    }
}

void getParameterInfo(ParameterIndex index, ParameterInfo * info) const {
    {
        switch (index) {
        case 0:
            {
            info->type = ParameterTypeNumber;
            info->initialValue = 3000;
            info->min = 20;
            info->max = 20000;
            info->exponent = 1;
            info->steps = 0;
            info->debug = false;
            info->saveable = true;
            info->transmittable = true;
            info->initialized = true;
            info->visible = true;
            info->displayName = "";
            info->unit = "";
            info->ioType = IOTypeUndefined;
            info->signalIndex = INVALID_INDEX;
            break;
            }
        }
    }
}

void sendParameter(ParameterIndex index, bool ignoreValue) {
    this->getEngine()->notifyParameterValueChanged(index, (ignoreValue ? 0 : this->getParameterValue(index)), ignoreValue);
}

ParameterValue applyStepsToNormalizedParameterValue(ParameterValue normalizedValue, int steps) const {
    if (steps == 1) {
        if (normalizedValue > 0) {
            normalizedValue = 1.;
        }
    } else {
        ParameterValue oneStep = (number)1. / (steps - 1);
        ParameterValue numberOfSteps = rnbo_fround(normalizedValue / oneStep * 1 / (number)1) * (number)1;
        normalizedValue = numberOfSteps * oneStep;
    }

    return normalizedValue;
}

ParameterValue convertToNormalizedParameterValue(ParameterIndex index, ParameterValue value) const {
    switch (index) {
    case 0:
        {
        {
            value = (value < 20 ? 20 : (value > 20000 ? 20000 : value));
            ParameterValue normalizedValue = (value - 20) / (20000 - 20);
            return normalizedValue;
        }
        }
    default:
        {
        return value;
        }
    }
}

ParameterValue convertFromNormalizedParameterValue(ParameterIndex index, ParameterValue value) const {
    value = (value < 0 ? 0 : (value > 1 ? 1 : value));

    switch (index) {
    case 0:
        {
        {
            value = (value < 0 ? 0 : (value > 1 ? 1 : value));

            {
                return 20 + value * (20000 - 20);
            }
        }
        }
    default:
        {
        return value;
        }
    }
}

ParameterValue constrainParameterValue(ParameterIndex index, ParameterValue value) const {
    switch (index) {
    case 0:
        {
        return this->param_01_value_constrain(value);
        }
    default:
        {
        return value;
        }
    }
}

void scheduleParamInit(ParameterIndex index, Index order) {
    this->paramInitIndices->push(index);
    this->paramInitOrder->push(order);
}

void processParamInitEvents() {
    this->listquicksort(
        this->paramInitOrder,
        this->paramInitIndices,
        0,
        (int)(this->paramInitOrder->length - 1),
        true
    );

    for (Index i = 0; i < this->paramInitOrder->length; i++) {
        this->getEngine()->scheduleParameterChange(
            this->paramInitIndices[i],
            this->getParameterValue(this->paramInitIndices[i]),
            0
        );
    }
}

void processClockEvent(MillisecondTime time, ClockId index, bool hasValue, ParameterValue value) {
    RNBO_UNUSED(value);
    RNBO_UNUSED(hasValue);
    this->updateTime(time);

    switch (index) {
    case 760652352:
        {
        this->line_01_target_bang();
        break;
        }
    }
}

void processOutletAtCurrentTime(EngineLink* , OutletIndex , ParameterValue ) {}

void processOutletEvent(
    EngineLink* sender,
    OutletIndex index,
    ParameterValue value,
    MillisecondTime time
) {
    this->updateTime(time);
    this->processOutletAtCurrentTime(sender, index, value);
}

void processNumMessage(MessageTag tag, MessageTag objectId, MillisecondTime time, number payload) {
    this->updateTime(time);

    switch (tag) {
    case TAG("valin"):
        {
        if (TAG("number_obj-37") == objectId)
            this->numberobj_01_valin_set(payload);

        break;
        }
    case TAG("format"):
        {
        if (TAG("number_obj-37") == objectId)
            this->numberobj_01_format_set(payload);

        break;
        }
    }
}

void processListMessage(MessageTag , MessageTag , MillisecondTime , const list& ) {}

void processBangMessage(MessageTag , MessageTag , MillisecondTime ) {}

MessageTagInfo resolveTag(MessageTag tag) const {
    switch (tag) {
    case TAG("valout"):
        {
        return "valout";
        }
    case TAG("number_obj-37"):
        {
        return "number_obj-37";
        }
    case TAG("setup"):
        {
        return "setup";
        }
    case TAG("valin"):
        {
        return "valin";
        }
    case TAG("format"):
        {
        return "format";
        }
    }

    return "";
}

MessageIndex getNumMessages() const {
    return 0;
}

const MessageInfo& getMessageInfo(MessageIndex index) const {
    switch (index) {

    }

    return NullMessageInfo;
}

protected:

void param_01_value_set(number v) {
    v = this->param_01_value_constrain(v);
    this->param_01_value = v;
    this->sendParameter(0, false);

    if (this->param_01_value != this->param_01_lastValue) {
        this->getEngine()->presetTouched();
        this->param_01_lastValue = this->param_01_value;
    }

    this->numberobj_01_value_set(v);
}

void numberobj_01_valin_set(number v) {
    this->numberobj_01_value_set(v);
}

void numberobj_01_format_set(number v) {
    this->numberobj_01_currentFormat = rnbo_trunc((v > 6 ? 6 : (v < 0 ? 0 : v)));
}

void line_01_target_bang() {}

number msToSamps(MillisecondTime ms, number sampleRate) {
    return ms * sampleRate * 0.001;
}

MillisecondTime sampsToMs(SampleIndex samps) {
    return samps * (this->invsr * 1000);
}

Index getMaxBlockSize() const {
    return this->maxvs;
}

number getSampleRate() const {
    return this->sr;
}

bool hasFixedVectorSize() const {
    return false;
}

Index getNumInputChannels() const {
    return 2;
}

Index getNumOutputChannels() const {
    return 2;
}

void allocateDataRefs() {}

void initializeObjects() {
    this->numberobj_01_init();
}

void sendOutlet(OutletIndex index, ParameterValue value) {
    this->getEngine()->sendOutlet(this, index, value);
}

void startup() {
    this->updateTime(this->getEngine()->getCurrentTime());

    {
        this->scheduleParamInit(0, 0);
    }

    this->processParamInitEvents();
}

static number param_01_value_constrain(number v) {
    v = (v > 20000 ? 20000 : (v < 20 ? 20 : v));
    return v;
}

void line_01_segments_set(const list& v) {
    this->line_01_segments = jsCreateListCopy(v);

    if ((bool)(v->length)) {
        auto currentTime = this->currentsampletime();
        number lastRampValue = this->line_01_currentValue;
        number rampEnd = currentTime - this->sampleOffsetIntoNextAudioBuffer;

        for (Index i = 0; i < this->line_01_activeRamps->length; i += 3) {
            rampEnd = this->line_01_activeRamps[(Index)(i + 2)];

            if (rampEnd > currentTime) {
                this->line_01_activeRamps[(Index)(i + 2)] = currentTime;
                number diff = rampEnd - currentTime;
                number valueDiff = diff * this->line_01_activeRamps[(Index)(i + 1)];
                lastRampValue = this->line_01_activeRamps[(Index)i] - valueDiff;
                this->line_01_activeRamps[(Index)i] = lastRampValue;
                this->line_01_activeRamps->length = i + 3;
                rampEnd = currentTime;
            } else {
                lastRampValue = this->line_01_activeRamps[(Index)i];
            }
        }

        if (rampEnd < currentTime) {
            this->line_01_activeRamps->push(lastRampValue);
            this->line_01_activeRamps->push(0);
            this->line_01_activeRamps->push(currentTime);
        }

        number lastRampEnd = currentTime;

        for (Index i = 0; i < v->length; i += 2) {
            number destinationValue = v[(Index)i];
            number inc = 0;
            number rampTimeInSamples;

            if (v->length > i + 1) {
                rampTimeInSamples = this->mstosamps(v[(Index)(i + 1)]);
            } else {
                rampTimeInSamples = this->mstosamps(this->line_01_time);
            }

            if (rampTimeInSamples <= 0)
                rampTimeInSamples = 1;

            inc = (destinationValue - lastRampValue) / rampTimeInSamples;
            lastRampEnd += rampTimeInSamples;
            this->line_01_activeRamps->push(destinationValue);
            this->line_01_activeRamps->push(inc);
            this->line_01_activeRamps->push(lastRampEnd);
            lastRampValue = destinationValue;
        }
    }
}

void numberobj_01_output_set(number v) {
    {
        list converted = {v};
        this->line_01_segments_set(converted);
    }
}

void numberobj_01_value_set(number v) {
    this->numberobj_01_value_setter(v);
    v = this->numberobj_01_value;
    number localvalue = v;

    if (this->numberobj_01_currentFormat != 6) {
        localvalue = rnbo_trunc(localvalue);
    }

    this->getEngine()->sendNumMessage(TAG("valout"), TAG("number_obj-37"), localvalue, this->_currentTime);
    this->numberobj_01_output_set(localvalue);
}

void line_01_perform(SampleValue * out, Index n) {
    auto __line_01_currentValue = this->line_01_currentValue;
    Index i = 0;

    if ((bool)(this->line_01_activeRamps->length)) {
        while ((bool)(this->line_01_activeRamps->length) && i < n) {
            number destinationValue = this->line_01_activeRamps[0];
            number inc = this->line_01_activeRamps[1];
            number rampTimeInSamples = this->line_01_activeRamps[2] - this->audioProcessSampleCount - i;
            number val = __line_01_currentValue;

            while (rampTimeInSamples > 0 && i < n) {
                out[(Index)i] = val;
                val += inc;
                i++;
                rampTimeInSamples--;
            }

            if (rampTimeInSamples <= 0) {
                val = destinationValue;
                this->line_01_activeRamps->splice(0, 3);

                if ((bool)(!(bool)(this->line_01_activeRamps->length))) this->getEngine()->scheduleClockEventWithValue(
                    this,
                    760652352,
                    this->sampsToMs((SampleIndex)(this->vs)) + this->_currentTime,
                    0
                );;
            }

            __line_01_currentValue = val;
        }
    }

    while (i < n) {
        out[(Index)i] = __line_01_currentValue;
        i++;
    }

    this->line_01_currentValue = __line_01_currentValue;
}

void filtercoeff_01_perform(
    const Sample * frequency,
    number gain,
    number q,
    SampleValue * a0,
    SampleValue * a1,
    SampleValue * a2,
    SampleValue * b1,
    SampleValue * b2,
    Index n
) {
    RNBO_UNUSED(q);
    RNBO_UNUSED(gain);
    auto __filtercoeff_01_activeResamp = this->filtercoeff_01_activeResamp;
    auto __filtercoeff_01_resamp_counter = this->filtercoeff_01_resamp_counter;
    auto __filtercoeff_01_K_EPSILON = this->filtercoeff_01_K_EPSILON;

    for (Index i = 0; i < n; i++) {
        number local_q = 1;
        number local_gain = 1;

        if (local_gain < 0)
            local_gain = 0;

        number local_frequency = frequency[(Index)i];

        if (local_q < __filtercoeff_01_K_EPSILON)
            local_q = __filtercoeff_01_K_EPSILON;

        local_frequency = (local_frequency > this->sr * 0.5 ? this->sr * 0.5 : (local_frequency < 1 ? 1 : local_frequency));
        __filtercoeff_01_resamp_counter--;

        if (__filtercoeff_01_resamp_counter <= 0) {
            __filtercoeff_01_resamp_counter = __filtercoeff_01_activeResamp;
            this->filtercoeff_01_updateTerms(local_frequency, local_gain, local_q);
        }

        a0[(Index)i] = this->filtercoeff_01_la0;
        a1[(Index)i] = this->filtercoeff_01_la1;
        a2[(Index)i] = this->filtercoeff_01_la2;
        b1[(Index)i] = this->filtercoeff_01_lb1;
        b2[(Index)i] = this->filtercoeff_01_lb2;
    }

    this->filtercoeff_01_resamp_counter = __filtercoeff_01_resamp_counter;
}

void biquad_tilde_02_perform(
    const Sample * x,
    const Sample * a0,
    const Sample * a1,
    const Sample * a2,
    const Sample * b1,
    const Sample * b2,
    SampleValue * out1,
    Index n
) {
    auto __biquad_tilde_02_y2 = this->biquad_tilde_02_y2;
    auto __biquad_tilde_02_y1 = this->biquad_tilde_02_y1;
    auto __biquad_tilde_02_x2 = this->biquad_tilde_02_x2;
    auto __biquad_tilde_02_x1 = this->biquad_tilde_02_x1;
    Index i;

    for (i = 0; i < n; i++) {
        number tmp = x[(Index)i] * a0[(Index)i] + __biquad_tilde_02_x1 * a1[(Index)i] + __biquad_tilde_02_x2 * a2[(Index)i] - (__biquad_tilde_02_y1 * b1[(Index)i] + __biquad_tilde_02_y2 * b2[(Index)i]);
        __biquad_tilde_02_x2 = __biquad_tilde_02_x1;
        __biquad_tilde_02_x1 = x[(Index)i];
        __biquad_tilde_02_y2 = __biquad_tilde_02_y1;
        __biquad_tilde_02_y1 = tmp;
        out1[(Index)i] = tmp;
    }

    this->biquad_tilde_02_x1 = __biquad_tilde_02_x1;
    this->biquad_tilde_02_x2 = __biquad_tilde_02_x2;
    this->biquad_tilde_02_y1 = __biquad_tilde_02_y1;
    this->biquad_tilde_02_y2 = __biquad_tilde_02_y2;
}

void biquad_tilde_01_perform(
    const Sample * x,
    const Sample * a0,
    const Sample * a1,
    const Sample * a2,
    const Sample * b1,
    const Sample * b2,
    SampleValue * out1,
    Index n
) {
    auto __biquad_tilde_01_y2 = this->biquad_tilde_01_y2;
    auto __biquad_tilde_01_y1 = this->biquad_tilde_01_y1;
    auto __biquad_tilde_01_x2 = this->biquad_tilde_01_x2;
    auto __biquad_tilde_01_x1 = this->biquad_tilde_01_x1;
    Index i;

    for (i = 0; i < n; i++) {
        number tmp = x[(Index)i] * a0[(Index)i] + __biquad_tilde_01_x1 * a1[(Index)i] + __biquad_tilde_01_x2 * a2[(Index)i] - (__biquad_tilde_01_y1 * b1[(Index)i] + __biquad_tilde_01_y2 * b2[(Index)i]);
        __biquad_tilde_01_x2 = __biquad_tilde_01_x1;
        __biquad_tilde_01_x1 = x[(Index)i];
        __biquad_tilde_01_y2 = __biquad_tilde_01_y1;
        __biquad_tilde_01_y1 = tmp;
        out1[(Index)i] = tmp;
    }

    this->biquad_tilde_01_x1 = __biquad_tilde_01_x1;
    this->biquad_tilde_01_x2 = __biquad_tilde_01_x2;
    this->biquad_tilde_01_y1 = __biquad_tilde_01_y1;
    this->biquad_tilde_01_y2 = __biquad_tilde_01_y2;
}

void stackprotect_perform(Index n) {
    RNBO_UNUSED(n);
    auto __stackprotect_count = this->stackprotect_count;
    __stackprotect_count = 0;
    this->stackprotect_count = __stackprotect_count;
}

void numberobj_01_value_setter(number v) {
    number localvalue = v;

    if (this->numberobj_01_currentFormat != 6) {
        localvalue = rnbo_trunc(localvalue);
    }

    this->numberobj_01_value = localvalue;
}

void biquad_tilde_01_reset() {
    this->biquad_tilde_01_x1 = 0;
    this->biquad_tilde_01_x2 = 0;
    this->biquad_tilde_01_y1 = 0;
    this->biquad_tilde_01_y2 = 0;
}

void biquad_tilde_01_dspsetup(bool force) {
    if ((bool)(this->biquad_tilde_01_setupDone) && (bool)(!(bool)(force)))
        return;

    this->biquad_tilde_01_reset();
    this->biquad_tilde_01_setupDone = true;
}

void biquad_tilde_02_reset() {
    this->biquad_tilde_02_x1 = 0;
    this->biquad_tilde_02_x2 = 0;
    this->biquad_tilde_02_y1 = 0;
    this->biquad_tilde_02_y2 = 0;
}

void biquad_tilde_02_dspsetup(bool force) {
    if ((bool)(this->biquad_tilde_02_setupDone) && (bool)(!(bool)(force)))
        return;

    this->biquad_tilde_02_reset();
    this->biquad_tilde_02_setupDone = true;
}

void numberobj_01_init() {
    this->numberobj_01_currentFormat = 6;
    this->getEngine()->sendNumMessage(TAG("setup"), TAG("number_obj-37"), 1, this->_currentTime);
}

void numberobj_01_getPresetValue(PatcherStateInterface& preset) {
    preset["value"] = this->numberobj_01_value;
}

void numberobj_01_setPresetValue(PatcherStateInterface& preset) {
    if ((bool)(stateIsEmpty(preset)))
        return;

    this->numberobj_01_value_set(preset["value"]);
}

void param_01_getPresetValue(PatcherStateInterface& preset) {
    preset["value"] = this->param_01_value;
}

void param_01_setPresetValue(PatcherStateInterface& preset) {
    if ((bool)(stateIsEmpty(preset)))
        return;

    this->param_01_value_set(preset["value"]);
}

array<number, 5> filtercoeff_01_localop_next(number frequency, number q, number gain, number type) {
    number omega = frequency * this->filtercoeff_01_localop_twopi_over_sr;
    this->filtercoeff_01_localop_cs = rnbo_cos(omega);
    this->filtercoeff_01_localop_sn = rnbo_sin(omega);
    this->filtercoeff_01_localop_one_over_gain = (gain >= 0 ? (number)1 / gain : 0.0);
    this->filtercoeff_01_localop_one_over_q = (number)1 / q;
    this->filtercoeff_01_localop_alpha = this->filtercoeff_01_localop_sn * 0.5 * this->filtercoeff_01_localop_one_over_q;

    switch ((int)type) {
    case 5:
        {
        this->filtercoeff_01_localop_A = this->safesqrt(gain);

        this->filtercoeff_01_localop_beta = this->safesqrt(
            (this->filtercoeff_01_localop_A * this->filtercoeff_01_localop_A + 1.) * this->filtercoeff_01_localop_one_over_q - (this->filtercoeff_01_localop_A - 1.) * (this->filtercoeff_01_localop_A - 1.)
        );

        this->filtercoeff_01_localop_b0 = (number)1 / (this->filtercoeff_01_localop_A + 1. + (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs + this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn);
        break;
        }
    case 6:
        {
        this->filtercoeff_01_localop_A = this->safesqrt(gain);

        this->filtercoeff_01_localop_beta = this->safesqrt(
            (this->filtercoeff_01_localop_A * this->filtercoeff_01_localop_A + 1.) * this->filtercoeff_01_localop_one_over_q - (this->filtercoeff_01_localop_A - 1.) * (this->filtercoeff_01_localop_A - 1.)
        );

        this->filtercoeff_01_localop_b0 = (number)1 / (this->filtercoeff_01_localop_A + 1. - (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs + this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn);
        break;
        }
    case 4:
        {
        this->filtercoeff_01_localop_A = this->safesqrt(gain);
        this->filtercoeff_01_localop_one_over_a = (this->filtercoeff_01_localop_A == 0 ? 0 : (number)1 / this->filtercoeff_01_localop_A);
        this->filtercoeff_01_localop_b0 = (number)1 / (1. + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_a);
        break;
        }
    case 9:
    case 10:
    case 11:
    case 13:
    case 14:
        {
        this->filtercoeff_01_localop_b0 = (number)1 / (1. + this->filtercoeff_01_localop_alpha);
        this->filtercoeff_01_localop_b0g = (number)1 / (this->filtercoeff_01_localop_one_over_gain + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_gain);
        break;
        }
    default:
        {
        this->filtercoeff_01_localop_b0 = (number)1 / (1. + this->filtercoeff_01_localop_alpha);
        break;
        }
    }

    switch ((int)type) {
    case 0:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_la2 = (1. - this->filtercoeff_01_localop_cs) * 0.5 * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = (1. - this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 1:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_la2 = (1. + this->filtercoeff_01_localop_cs) * 0.5 * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = -(1. + this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 2:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = 0.;
        this->filtercoeff_01_localop_la2 = -this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 7:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_alpha * q * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = 0.;
        this->filtercoeff_01_localop_la2 = -this->filtercoeff_01_localop_alpha * q * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 3:
        {
        this->filtercoeff_01_localop_la1 = this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_la2 = this->filtercoeff_01_localop_b0;
        break;
        }
    case 8:
        {
        this->filtercoeff_01_localop_la1 = this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = this->filtercoeff_01_localop_la0 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la2 = 1.0;
        break;
        }
    case 4:
        {
        this->filtercoeff_01_localop_la0 = (1. + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_A) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la2 = (1. - this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_A) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_a) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 5:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_A * (this->filtercoeff_01_localop_A + 1. - (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs + this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = 2. * this->filtercoeff_01_localop_A * (this->filtercoeff_01_localop_A - 1 - (this->filtercoeff_01_localop_A + 1) * this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la2 = this->filtercoeff_01_localop_A * (this->filtercoeff_01_localop_A + 1. - (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs - this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * (this->filtercoeff_01_localop_A - 1. + (this->filtercoeff_01_localop_A + 1.) * this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (this->filtercoeff_01_localop_A + 1. + (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs - this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 6:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_A * (this->filtercoeff_01_localop_A + 1. + (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs + this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = -2. * this->filtercoeff_01_localop_A * (this->filtercoeff_01_localop_A - 1. + (this->filtercoeff_01_localop_A + 1.) * this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la2 = this->filtercoeff_01_localop_A * (this->filtercoeff_01_localop_A + 1. + (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs - this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = 2. * (this->filtercoeff_01_localop_A - 1. - (this->filtercoeff_01_localop_A + 1.) * this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (this->filtercoeff_01_localop_A + 1. - (this->filtercoeff_01_localop_A - 1.) * this->filtercoeff_01_localop_cs - this->filtercoeff_01_localop_beta * this->filtercoeff_01_localop_sn) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 9:
        {
        this->filtercoeff_01_localop_b0g = (number)1 / (this->filtercoeff_01_localop_one_over_gain + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_gain);
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_la2 = (1. - this->filtercoeff_01_localop_cs) * 0.5 * this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_la1 = (1. - this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 10:
        {
        this->filtercoeff_01_localop_b0g = (number)1 / (this->filtercoeff_01_localop_one_over_gain + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_gain);
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_la2 = (1. + this->filtercoeff_01_localop_cs) * 0.5 * this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_la1 = -(1. + this->filtercoeff_01_localop_cs) * this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 11:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_alpha * gain * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = 0.;
        this->filtercoeff_01_localop_la2 = -this->filtercoeff_01_localop_alpha * gain * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 13:
        {
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_alpha * gain * q * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 = 0.;
        this->filtercoeff_01_localop_la2 = -this->filtercoeff_01_localop_alpha * gain * q * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 12:
        {
        this->filtercoeff_01_localop_b0g = (number)1 / (this->filtercoeff_01_localop_one_over_gain + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_gain);
        this->filtercoeff_01_localop_la1 = this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la1 *= this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_lb1 *= this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_la0 = this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_la2 = this->filtercoeff_01_localop_b0g;
        break;
        }
    case 14:
        {
        this->filtercoeff_01_localop_b0g = (number)1 / (this->filtercoeff_01_localop_one_over_gain + this->filtercoeff_01_localop_alpha * this->filtercoeff_01_localop_one_over_gain);
        this->filtercoeff_01_localop_la0 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_la1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0g;
        this->filtercoeff_01_localop_la2 = gain;
        this->filtercoeff_01_localop_lb1 = -2. * this->filtercoeff_01_localop_cs * this->filtercoeff_01_localop_b0;
        this->filtercoeff_01_localop_lb2 = (1. - this->filtercoeff_01_localop_alpha) * this->filtercoeff_01_localop_b0;
        break;
        }
    case 15:
        {
        this->filtercoeff_01_localop_la0 = 1;
        this->filtercoeff_01_localop_la1 = 0;
        this->filtercoeff_01_localop_la2 = 0;
        this->filtercoeff_01_localop_lb1 = 0;
        this->filtercoeff_01_localop_lb2 = 0;
        }
    default:
        {
        break;
        }
    }

    return {
        this->filtercoeff_01_localop_la0,
        this->filtercoeff_01_localop_la1,
        this->filtercoeff_01_localop_la2,
        this->filtercoeff_01_localop_lb1,
        this->filtercoeff_01_localop_lb2
    };
}

void filtercoeff_01_localop_dspsetup() {
    this->filtercoeff_01_localop_twopi_over_sr = (number)6.283185307179586 / this->sr;
}

void filtercoeff_01_localop_reset() {
    this->filtercoeff_01_localop_internal = true;
    this->filtercoeff_01_localop_twopi_over_sr = 0;
    this->filtercoeff_01_localop_cs = 0;
    this->filtercoeff_01_localop_sn = 0;
    this->filtercoeff_01_localop_one_over_gain = 0;
    this->filtercoeff_01_localop_one_over_q = 0;
    this->filtercoeff_01_localop_alpha = 0;
    this->filtercoeff_01_localop_beta = 0;
    this->filtercoeff_01_localop_b0 = 0;
    this->filtercoeff_01_localop_b0g = 0;
    this->filtercoeff_01_localop_A = 0;
    this->filtercoeff_01_localop_one_over_a = 0;
    this->filtercoeff_01_localop_la0 = 0;
    this->filtercoeff_01_localop_la1 = 0;
    this->filtercoeff_01_localop_la2 = 0;
    this->filtercoeff_01_localop_lb1 = 0;
    this->filtercoeff_01_localop_lb2 = 0;
}

void filtercoeff_01_updateTerms(number local_frequency, number local_gain, number local_q) {
    array<number, 5> tmp = this->filtercoeff_01_localop_next(local_frequency, local_q, local_gain, this->filtercoeff_01_type);
    this->filtercoeff_01_la0 = tmp[0];
    this->filtercoeff_01_la1 = tmp[1];
    this->filtercoeff_01_la2 = tmp[2];
    this->filtercoeff_01_lb1 = tmp[3];
    this->filtercoeff_01_lb2 = tmp[4];
}

void filtercoeff_01_dspsetup(bool force) {
    if ((bool)(this->filtercoeff_01_setupDone) && (bool)(!(bool)(force)))
        return;

    {
        this->filtercoeff_01_activeResamp = this->vectorsize();
    }

    this->filtercoeff_01_resamp_counter = 0;
    this->filtercoeff_01_setupDone = true;
    this->filtercoeff_01_localop_dspsetup();
}

number globaltransport_getTempoAtSample(SampleIndex sampleOffset) {
    RNBO_UNUSED(sampleOffset);
    return (this->vs > 0 ? this->globaltransport_tempo[(Index)sampleOffset] : this->globaltransport_lastTempo);
}

number globaltransport_getTempo() {
    return this->globaltransport_getTempoAtSample(this->sampleOffsetIntoNextAudioBuffer);
}

number globaltransport_getStateAtSample(SampleIndex sampleOffset) {
    RNBO_UNUSED(sampleOffset);
    return (this->vs > 0 ? this->globaltransport_state[(Index)sampleOffset] : this->globaltransport_lastState);
}

number globaltransport_getState() {
    return this->globaltransport_getStateAtSample(this->sampleOffsetIntoNextAudioBuffer);
}

number globaltransport_getBeatTimeAtMsTime(MillisecondTime time) {
    number i = 2;

    while (i < this->globaltransport_beatTimeChanges->length && this->globaltransport_beatTimeChanges[(Index)(i + 1)] <= time) {
        i += 2;
    }

    i -= 2;
    number beatTimeBase = this->globaltransport_beatTimeChanges[(Index)i];

    if (this->globaltransport_getState() == 0)
        return beatTimeBase;

    number beatTimeBaseMsTime = this->globaltransport_beatTimeChanges[(Index)(i + 1)];
    number diff = time - beatTimeBaseMsTime;
    return beatTimeBase + this->mstobeats(diff);
}

bool globaltransport_setTempo(number tempo, bool notify) {
    if ((bool)(notify)) {
        this->processTempoEvent(this->currenttime(), tempo);
        this->globaltransport_notify = true;
    } else if (this->globaltransport_getTempo() != tempo) {
        auto ct = this->currenttime();
        this->globaltransport_beatTimeChanges->push(this->globaltransport_getBeatTimeAtMsTime(ct));
        this->globaltransport_beatTimeChanges->push(ct);

        fillSignal(
            this->globaltransport_tempo,
            this->vs,
            tempo,
            (Index)(this->sampleOffsetIntoNextAudioBuffer)
        );

        this->globaltransport_lastTempo = tempo;
        this->globaltransport_tempoNeedsReset = true;
        return true;
    }

    return false;
}

number globaltransport_getBeatTime() {
    return this->globaltransport_getBeatTimeAtMsTime(this->currenttime());
}

bool globaltransport_setState(number state, bool notify) {
    if ((bool)(notify)) {
        this->processTransportEvent(this->currenttime(), TransportState(state));
        this->globaltransport_notify = true;
    } else if (this->globaltransport_getState() != state) {
        fillSignal(
            this->globaltransport_state,
            this->vs,
            state,
            (Index)(this->sampleOffsetIntoNextAudioBuffer)
        );

        this->globaltransport_lastState = TransportState(state);
        this->globaltransport_stateNeedsReset = true;

        if (state == 0) {
            this->globaltransport_beatTimeChanges->push(this->globaltransport_getBeatTime());
            this->globaltransport_beatTimeChanges->push(this->currenttime());
        }

        return true;
    }

    return false;
}

bool globaltransport_setBeatTime(number beattime, bool notify) {
    if ((bool)(notify)) {
        this->processBeatTimeEvent(this->currenttime(), beattime);
        this->globaltransport_notify = true;
        return false;
    } else {
        bool beatTimeHasChanged = false;
        float oldBeatTime = (float)(this->globaltransport_getBeatTime());
        float newBeatTime = (float)(beattime);

        if (oldBeatTime != newBeatTime) {
            beatTimeHasChanged = true;
        }

        this->globaltransport_beatTimeChanges->push(beattime);
        this->globaltransport_beatTimeChanges->push(this->currenttime());
        return beatTimeHasChanged;
    }
}

number globaltransport_getBeatTimeAtSample(SampleIndex sampleOffset) {
    auto msOffset = this->sampstoms(sampleOffset);
    return this->globaltransport_getBeatTimeAtMsTime(this->currenttime() + msOffset);
}

array<number, 2> globaltransport_getTimeSignatureAtMsTime(MillisecondTime time) {
    number i = 3;

    while (i < this->globaltransport_timeSignatureChanges->length && this->globaltransport_timeSignatureChanges[(Index)(i + 2)] <= time) {
        i += 3;
    }

    i -= 3;

    return {
        this->globaltransport_timeSignatureChanges[(Index)i],
        this->globaltransport_timeSignatureChanges[(Index)(i + 1)]
    };
}

array<number, 2> globaltransport_getTimeSignature() {
    return this->globaltransport_getTimeSignatureAtMsTime(this->currenttime());
}

array<number, 2> globaltransport_getTimeSignatureAtSample(SampleIndex sampleOffset) {
    auto msOffset = this->sampstoms(sampleOffset);
    return this->globaltransport_getTimeSignatureAtMsTime(this->currenttime() + msOffset);
}

bool globaltransport_setTimeSignature(number numerator, number denominator, bool notify) {
    if ((bool)(notify)) {
        this->processTimeSignatureEvent(this->currenttime(), (int)(numerator), (int)(denominator));
        this->globaltransport_notify = true;
    } else {
        array<number, 2> currentSig = this->globaltransport_getTimeSignature();

        if (currentSig[0] != numerator || currentSig[1] != denominator) {
            this->globaltransport_timeSignatureChanges->push(numerator);
            this->globaltransport_timeSignatureChanges->push(denominator);
            this->globaltransport_timeSignatureChanges->push(this->currenttime());
            return true;
        }
    }

    return false;
}

void globaltransport_advance() {
    if ((bool)(this->globaltransport_tempoNeedsReset)) {
        fillSignal(this->globaltransport_tempo, this->vs, this->globaltransport_lastTempo);
        this->globaltransport_tempoNeedsReset = false;

        if ((bool)(this->globaltransport_notify)) {
            this->getEngine()->sendTempoEvent(this->globaltransport_lastTempo);
        }
    }

    if ((bool)(this->globaltransport_stateNeedsReset)) {
        fillSignal(this->globaltransport_state, this->vs, this->globaltransport_lastState);
        this->globaltransport_stateNeedsReset = false;

        if ((bool)(this->globaltransport_notify)) {
            this->getEngine()->sendTransportEvent(TransportState(this->globaltransport_lastState));
        }
    }

    if (this->globaltransport_beatTimeChanges->length > 2) {
        this->globaltransport_beatTimeChanges[0] = this->globaltransport_beatTimeChanges[(Index)(this->globaltransport_beatTimeChanges->length - 2)];
        this->globaltransport_beatTimeChanges[1] = this->globaltransport_beatTimeChanges[(Index)(this->globaltransport_beatTimeChanges->length - 1)];
        this->globaltransport_beatTimeChanges->length = 2;

        if ((bool)(this->globaltransport_notify)) {
            this->getEngine()->sendBeatTimeEvent(this->globaltransport_beatTimeChanges[0]);
        }
    }

    if (this->globaltransport_timeSignatureChanges->length > 3) {
        this->globaltransport_timeSignatureChanges[0] = this->globaltransport_timeSignatureChanges[(Index)(this->globaltransport_timeSignatureChanges->length - 3)];
        this->globaltransport_timeSignatureChanges[1] = this->globaltransport_timeSignatureChanges[(Index)(this->globaltransport_timeSignatureChanges->length - 2)];
        this->globaltransport_timeSignatureChanges[2] = this->globaltransport_timeSignatureChanges[(Index)(this->globaltransport_timeSignatureChanges->length - 1)];
        this->globaltransport_timeSignatureChanges->length = 3;

        if ((bool)(this->globaltransport_notify)) {
            this->getEngine()->sendTimeSignatureEvent(
                (int)(this->globaltransport_timeSignatureChanges[0]),
                (int)(this->globaltransport_timeSignatureChanges[1])
            );
        }
    }

    this->globaltransport_notify = false;
}

void globaltransport_dspsetup(bool force) {
    if ((bool)(this->globaltransport_setupDone) && (bool)(!(bool)(force)))
        return;

    fillSignal(this->globaltransport_tempo, this->vs, this->globaltransport_lastTempo);
    this->globaltransport_tempoNeedsReset = false;
    fillSignal(this->globaltransport_state, this->vs, this->globaltransport_lastState);
    this->globaltransport_stateNeedsReset = false;
    this->globaltransport_setupDone = true;
}

bool stackprotect_check() {
    this->stackprotect_count++;

    if (this->stackprotect_count > 128) {
        console->log("STACK OVERFLOW DETECTED - stopped processing branch !");
        return true;
    }

    return false;
}

void updateTime(MillisecondTime time) {
    this->_currentTime = time;
    this->sampleOffsetIntoNextAudioBuffer = (SampleIndex)(rnbo_fround(this->msToSamps(time - this->getEngine()->getCurrentTime(), this->sr)));

    if (this->sampleOffsetIntoNextAudioBuffer >= (SampleIndex)(this->vs))
        this->sampleOffsetIntoNextAudioBuffer = (SampleIndex)(this->vs) - 1;

    if (this->sampleOffsetIntoNextAudioBuffer < 0)
        this->sampleOffsetIntoNextAudioBuffer = 0;
}

void assign_defaults()
{
    biquad_tilde_01_x = 0;
    biquad_tilde_01_a0 = 0;
    biquad_tilde_01_a1 = 0;
    biquad_tilde_01_a2 = 0;
    biquad_tilde_01_b1 = 0;
    biquad_tilde_01_b2 = 0;
    biquad_tilde_02_x = 0;
    biquad_tilde_02_a0 = 0;
    biquad_tilde_02_a1 = 0;
    biquad_tilde_02_a2 = 0;
    biquad_tilde_02_b1 = 0;
    biquad_tilde_02_b2 = 0;
    numberobj_01_value = 0;
    numberobj_01_value_setter(numberobj_01_value);
    param_01_value = 3000;
    line_01_time = 10;
    filtercoeff_01_frequency = 1000;
    filtercoeff_01_gain = 1;
    filtercoeff_01_q = 1;
    filtercoeff_01_type = 0;
    _currentTime = 0;
    audioProcessSampleCount = 0;
    sampleOffsetIntoNextAudioBuffer = 0;
    zeroBuffer = nullptr;
    dummyBuffer = nullptr;
    signals[0] = nullptr;
    signals[1] = nullptr;
    signals[2] = nullptr;
    signals[3] = nullptr;
    signals[4] = nullptr;
    signals[5] = nullptr;
    didAllocateSignals = 0;
    vs = 0;
    maxvs = 0;
    sr = 44100;
    invsr = 0.00002267573696;
    biquad_tilde_01_x1 = 0;
    biquad_tilde_01_x2 = 0;
    biquad_tilde_01_y1 = 0;
    biquad_tilde_01_y2 = 0;
    biquad_tilde_01_setupDone = false;
    biquad_tilde_02_x1 = 0;
    biquad_tilde_02_x2 = 0;
    biquad_tilde_02_y1 = 0;
    biquad_tilde_02_y2 = 0;
    biquad_tilde_02_setupDone = false;
    numberobj_01_currentFormat = 6;
    numberobj_01_lastValue = 0;
    param_01_lastValue = 0;
    line_01_currentValue = 20000;
    filtercoeff_01_K_EPSILON = 1e-9;
    filtercoeff_01_localop_internal = true;
    filtercoeff_01_setupDone = false;
    globaltransport_tempo = nullptr;
    globaltransport_tempoNeedsReset = false;
    globaltransport_lastTempo = 120;
    globaltransport_state = nullptr;
    globaltransport_stateNeedsReset = false;
    globaltransport_lastState = 0;
    globaltransport_beatTimeChanges = { 0, 0 };
    globaltransport_timeSignatureChanges = { 4, 4, 0 };
    globaltransport_notify = false;
    globaltransport_setupDone = false;
    stackprotect_count = 0;
    _voiceIndex = 0;
    _noteNumber = 0;
    isMuted = 1;
}

// member variables

    number biquad_tilde_01_x;
    number biquad_tilde_01_a0;
    number biquad_tilde_01_a1;
    number biquad_tilde_01_a2;
    number biquad_tilde_01_b1;
    number biquad_tilde_01_b2;
    number biquad_tilde_02_x;
    number biquad_tilde_02_a0;
    number biquad_tilde_02_a1;
    number biquad_tilde_02_a2;
    number biquad_tilde_02_b1;
    number biquad_tilde_02_b2;
    number numberobj_01_value;
    number param_01_value;
    list line_01_segments;
    number line_01_time;
    number filtercoeff_01_frequency;
    number filtercoeff_01_gain;
    number filtercoeff_01_q;
    Int filtercoeff_01_type;
    MillisecondTime _currentTime;
    SampleIndex audioProcessSampleCount;
    SampleIndex sampleOffsetIntoNextAudioBuffer;
    signal zeroBuffer;
    signal dummyBuffer;
    SampleValue * signals[6];
    bool didAllocateSignals;
    Index vs;
    Index maxvs;
    number sr;
    number invsr;
    number biquad_tilde_01_x1;
    number biquad_tilde_01_x2;
    number biquad_tilde_01_y1;
    number biquad_tilde_01_y2;
    bool biquad_tilde_01_setupDone;
    number biquad_tilde_02_x1;
    number biquad_tilde_02_x2;
    number biquad_tilde_02_y1;
    number biquad_tilde_02_y2;
    bool biquad_tilde_02_setupDone;
    Int numberobj_01_currentFormat;
    number numberobj_01_lastValue;
    number param_01_lastValue;
    list line_01_activeRamps;
    number line_01_currentValue;
    number filtercoeff_01_resamp_counter;
    number filtercoeff_01_activeResamp;
    number filtercoeff_01_K_EPSILON;
    number filtercoeff_01_la0;
    number filtercoeff_01_la1;
    number filtercoeff_01_la2;
    number filtercoeff_01_lb1;
    number filtercoeff_01_lb2;
    bool filtercoeff_01_localop_internal;
    number filtercoeff_01_localop_twopi_over_sr;
    number filtercoeff_01_localop_cs;
    number filtercoeff_01_localop_sn;
    number filtercoeff_01_localop_one_over_gain;
    number filtercoeff_01_localop_one_over_q;
    number filtercoeff_01_localop_alpha;
    number filtercoeff_01_localop_beta;
    number filtercoeff_01_localop_b0;
    number filtercoeff_01_localop_b0g;
    number filtercoeff_01_localop_A;
    number filtercoeff_01_localop_one_over_a;
    number filtercoeff_01_localop_la0;
    number filtercoeff_01_localop_la1;
    number filtercoeff_01_localop_la2;
    number filtercoeff_01_localop_lb1;
    number filtercoeff_01_localop_lb2;
    bool filtercoeff_01_setupDone;
    signal globaltransport_tempo;
    bool globaltransport_tempoNeedsReset;
    number globaltransport_lastTempo;
    signal globaltransport_state;
    bool globaltransport_stateNeedsReset;
    number globaltransport_lastState;
    list globaltransport_beatTimeChanges;
    list globaltransport_timeSignatureChanges;
    bool globaltransport_notify;
    bool globaltransport_setupDone;
    number stackprotect_count;
    Index _voiceIndex;
    Int _noteNumber;
    Index isMuted;
    indexlist paramInitIndices;
    indexlist paramInitOrder;

};

PatcherInterface* creaternbomatic()
{
    return new rnbomatic();
}

#ifndef RNBO_NO_PATCHERFACTORY

extern "C" PatcherFactoryFunctionPtr GetPatcherFactoryFunction(PlatformInterface* platformInterface)
#else

extern "C" PatcherFactoryFunctionPtr rnbomaticFactoryFunction(PlatformInterface* platformInterface)
#endif

{
    Platform::set(platformInterface);
    return creaternbomatic;
}

} // end RNBO namespace

