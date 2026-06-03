#include "JitterMonitor.h"

JitterMonitor::JitterMonitor()
    : _name("unnamed"),
      _expectedPeriodUs(1000),
      _lastStartUs(0),
      _mux(portMUX_INITIALIZER_UNLOCKED)
{
    resetAll();
}

JitterMonitor::JitterMonitor(const char* name, uint32_t expectedPeriodUs)
    : _name(name),
      _expectedPeriodUs(expectedPeriodUs),
      _lastStartUs(0),
      _mux(portMUX_INITIALIZER_UNLOCKED)
{
    resetAll();
}

void JitterMonitor::begin(const char* name, uint32_t expectedPeriodUs)
{
    portENTER_CRITICAL(&_mux);
    _name = name;
    _expectedPeriodUs = expectedPeriodUs;
    _lastStartUs = 0;
    resetAccumNoLock();
    resetReportNoLock();
    portEXIT_CRITICAL(&_mux);
}

uint32_t JitterMonitor::start()
{
    uint32_t nowUs = micros();

    portENTER_CRITICAL(&_mux);

    if (_lastStartUs != 0) {
        uint32_t periodUs = (uint32_t)(nowUs - _lastStartUs);
        uint32_t absJitterUs = (periodUs > _expectedPeriodUs) ?
                               (periodUs - _expectedPeriodUs) :
                               (_expectedPeriodUs - periodUs);

        _samples++;
        if (periodUs < _minPeriodUs) _minPeriodUs = periodUs;
        if (periodUs > _maxPeriodUs) _maxPeriodUs = periodUs;
        if (absJitterUs > _maxAbsJitterUs) _maxAbsJitterUs = absJitterUs;
        _sumPeriodUs += periodUs;
        _sumAbsJitterUs += absJitterUs;
    }

    _lastStartUs = nowUs;

    portEXIT_CRITICAL(&_mux);
    return nowUs;
}

void JitterMonitor::end(uint32_t startUs)
{
    uint32_t nowUs = micros();
    uint32_t execUs = (uint32_t)(nowUs - startUs);

    portENTER_CRITICAL(&_mux);

    _execSamples++;
    if (execUs < _minExecUs) _minExecUs = execUs;
    if (execUs > _maxExecUs) _maxExecUs = execUs;
    _sumExecUs += execUs;

    if (execUs > _expectedPeriodUs) {
        _overruns++;
    }

    portEXIT_CRITICAL(&_mux);
}

void JitterMonitor::updateReportAndReset()
{
    portENTER_CRITICAL(&_mux);

    _report.name = _name;
    _report.expectedPeriodUs = _expectedPeriodUs;

    _report.samples = _samples;
    _report.minPeriodUs = (_samples > 0) ? _minPeriodUs : 0;
    _report.maxPeriodUs = (_samples > 0) ? _maxPeriodUs : 0;
    _report.avgPeriodUs = (_samples > 0) ? ((float)_sumPeriodUs / (float)_samples) : 0.0f;

    _report.maxAbsJitterUs = (_samples > 0) ? _maxAbsJitterUs : 0;
    _report.avgAbsJitterUs = (_samples > 0) ? ((float)_sumAbsJitterUs / (float)_samples) : 0.0f;

    _report.minExecUs = (_execSamples > 0) ? _minExecUs : 0;
    _report.maxExecUs = (_execSamples > 0) ? _maxExecUs : 0;
    _report.avgExecUs = (_execSamples > 0) ? ((float)_sumExecUs / (float)_execSamples) : 0.0f;

    _report.overruns = _overruns;

    resetAccumNoLock();

    portEXIT_CRITICAL(&_mux);
}

JitterMonitor::Snapshot JitterMonitor::getReport() const
{
    Snapshot copy;

    portENTER_CRITICAL(&_mux);
    copy = _report;
    portEXIT_CRITICAL(&_mux);

    return copy;
}

void JitterMonitor::printReport(Stream& out) const
{
    Snapshot s = getReport();

    out.printf(
        "%s exp=%luus avgDt=%7.1f minDt=%5lu maxDt=%5lu avgJ=%6.1f maxJ=%5lu "
        "avgExec=%6.1f minExec=%4lu maxExec=%4lu over=%lu n=%lu",
        s.name,
        (unsigned long)s.expectedPeriodUs,
        s.avgPeriodUs,
        (unsigned long)s.minPeriodUs,
        (unsigned long)s.maxPeriodUs,
        s.avgAbsJitterUs,
        (unsigned long)s.maxAbsJitterUs,
        s.avgExecUs,
        (unsigned long)s.minExecUs,
        (unsigned long)s.maxExecUs,
        (unsigned long)s.overruns,
        (unsigned long)s.samples
    );
}

void JitterMonitor::resetAll()
{
    portENTER_CRITICAL(&_mux);
    _lastStartUs = 0;
    resetAccumNoLock();
    resetReportNoLock();
    portEXIT_CRITICAL(&_mux);
}

void JitterMonitor::resetAccumNoLock()
{
    _samples = 0;
    _minPeriodUs = UINT32_MAX;
    _maxPeriodUs = 0;
    _maxAbsJitterUs = 0;
    _sumPeriodUs = 0;
    _sumAbsJitterUs = 0;

    _execSamples = 0;
    _minExecUs = UINT32_MAX;
    _maxExecUs = 0;
    _sumExecUs = 0;
    _overruns = 0;
}

void JitterMonitor::resetReportNoLock()
{
    _report.name = _name;
    _report.expectedPeriodUs = _expectedPeriodUs;
    _report.samples = 0;
    _report.minPeriodUs = 0;
    _report.maxPeriodUs = 0;
    _report.avgPeriodUs = 0.0f;
    _report.maxAbsJitterUs = 0;
    _report.avgAbsJitterUs = 0.0f;
    _report.minExecUs = 0;
    _report.maxExecUs = 0;
    _report.avgExecUs = 0.0f;
    _report.overruns = 0;
}
