// Prints packet error rate vs SNR from ns-3's own NistErrorRateModel (the model the
// Yans PHY uses), for the data and control modes in the experiment YAML.
// Used by tools/link_budget.py to predict the cutoff distance BEFORE any sweep.
//   per_table <data_mode> <control_mode> <data_mpdu_bytes> <ack_bytes> [ErrorRateModelName=TableBased]   > per.csv
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <iostream>

using namespace ns3;

namespace
{
// P(frame ok) = P(PLCP header ok at its own mode) * P(payload ok at data mode)
double
FrameSuccess(Ptr<ErrorRateModel> m, const std::string& modeName, uint32_t mpduBytes, double snrLin)
{
    WifiMode mode = WifiMode(modeName);
    WifiTxVector tv(mode, 0, WIFI_PREAMBLE_LONG, NanoSeconds(800), 1, 1, 0, MHz_u{20}, false, false);
    WifiMode hdr = WifiMode("OfdmRate6Mbps");
    double ph = m->GetChunkSuccessRate(hdr, tv, snrLin, 24, 1, WIFI_PPDU_FIELD_NON_HT_HEADER);
    double pd = m->GetChunkSuccessRate(mode, tv, snrLin, 8ULL * mpduBytes + 22, 1, WIFI_PPDU_FIELD_DATA);
    return ph * pd;
}
} // namespace

int
main(int argc, char* argv[])
{
    if (argc < 5)
    {
        std::cerr << "usage: per_table <data_mode> <control_mode> <data_mpdu_bytes> <ack_bytes>\n";
        return 1;
    }
    std::string dataMode = argv[1];
    std::string ctrlMode = argv[2];
    uint32_t dataBytes = std::stoul(argv[3]);
    uint32_t ackBytes = std::stoul(argv[4]);
    // Must be the model the PHY actually uses (YansWifiPhyHelper default: TableBased, NOT Nist).
    std::string modelName = argc > 5 ? argv[5] : "TableBased";
    ObjectFactory f;
    f.SetTypeId("ns3::" + modelName + "ErrorRateModel");
    Ptr<ErrorRateModel> m = f.Create<ErrorRateModel>();
    std::cout << "snr_db,per_data,per_ack\n";
    for (double snr = -5; snr <= 30.001; snr += 0.25)
    {
        double lin = std::pow(10.0, snr / 10.0);
        std::cout << snr << "," << 1.0 - FrameSuccess(m, dataMode, dataBytes, lin) << ","
                  << 1.0 - FrameSuccess(m, ctrlMode, ackBytes, lin) << "\n";
    }
    return 0;
}
