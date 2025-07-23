#include "LPFrd.h"


////////////////////////////////////////////////////////////////////////////////////////////
// DigitalBiquadFilter with reduced delay modification
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalBiquadFilterRD<T>::DigitalBiquadFilterRD() {
  _delay_element_1 = T();
  _delay_element_2 = T();
}

template <class T>
T DigitalBiquadFilterRD<T>::apply(const T &sample, const struct biquad_params &params) {
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
        return sample;
    }

    T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

template <class T>
void DigitalBiquadFilterRD<T>::reset() { 
    _delay_element_1 = _delay_element_2 = T();
}

template <class T>
void DigitalBiquadFilterRD<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    if (!is_positive(ret.cutoff_freq)) {
        // zero cutoff means pass-thru
        return;
    }

    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    float tb0, tb1, tb2;
    tb0 = ohm*ohm/c;
    tb1 = 2.0f*tb0;
    tb2 = tb0;

    ret.a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret.a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;

    ret.b0 = tb0 + tb1 - tb0*ret.a1;
    ret.b1 = tb2 - tb0*(ret.a2 - ret.a1);
    ret.b2 = tb0*ret.a2;

    // printf("b0: %5.8f \n",ret.b0);
    // printf("b1: %5.8f \n",ret.b1);
    // printf("b2: %5.8f \n",ret.b2);

    // printf("a1: %5.8f \n",ret.a1);
    // printf("a2: %5.8f \n",ret.a2);
}


////////////////////////////////////////////////////////////////////////////////////////////
// Lowpass Filter with reduced delay modification
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
LPFrd<T>::LPFrd() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
LPFrd<T>::LPFrd(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void LPFrd<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    DigitalBiquadFilterRD<T>::compute_params(sample_freq, cutoff_freq, _params);
}

// return the cutoff frequency
template <class T>
float LPFrd<T>::get_cutoff_freq(void) const {
    return _params.cutoff_freq;
}

template <class T>
float LPFrd<T>::get_sample_freq(void) const {
    return _params.sample_freq;
}

template <class T>
T LPFrd<T>::apply(const T &sample) {
    if (!is_positive(_params.cutoff_freq)) {
        // zero cutoff means pass-thru
        return sample;
    }
    return _filter.apply(sample, _params);
}

template <class T>
void LPFrd<T>::reset(void) {
    return _filter.reset();
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LPFrd<int>;
template class LPFrd<long>;
template class LPFrd<float>;
template class LPFrd<Vector2f>;
template class LPFrd<Vector3f>;