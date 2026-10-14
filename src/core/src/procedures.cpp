// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/procedures.hpp"

namespace avd {

void ProcedureRunner::start(std::vector<Step> steps) {
    steps_ = std::move(steps);
    index_ = 0;
    elapsed_ = 0;
    status_ = steps_.empty() ? Status::Done : Status::Running;
    message_ = steps_.empty() ? "" : steps_[0].description;
}

void ProcedureRunner::update(Aircraft& ac, PilotInput& input, Real dt) {
    if (status_ != Status::Running) return;
    Step& s = steps_[index_];
    if (s.action) s.action(ac, input);
    elapsed_ += dt;
    if (!s.complete || s.complete(ac, elapsed_)) {
        ++index_;
        elapsed_ = 0;
        if (index_ >= static_cast<int>(steps_.size())) {
            status_ = Status::Done;
            message_ = "complete";
        } else {
            message_ = steps_[index_].description;
        }
        return;
    }
    if (elapsed_ > s.timeout) {
        status_ = Status::Failed;
        message_ = s.failure.empty() ? s.description + ": timed out" : s.failure;
    }
}

std::vector<ProcedureRunner::Step> quickStartPistonSingle(const Aircraft& ac) {
    using Step = ProcedureRunner::Step;
    std::vector<Step> v;
    auto set = [](const char* id, int pos) {
        return [id = std::string(id), pos](Aircraft& a, PilotInput&) { a.setCockpitControl(id, pos); };
    };
    int both = 0;
    const auto& sel = ac.definition().fuel.selector;
    for (std::size_t i = 0; i < sel.size(); ++i)
        if (sel[i].name == "BOTH") both = static_cast<int>(i);

    v.push_back({"Parking brake SET", set("brakes.parking", 1), nullptr, 5, ""});
    v.push_back({"Battery master ON", set("electrical.battery", 1),
                 [](const Aircraft& a, Real) { return a.electrical().busVoltage() > 20; }, 3,
                 "Battery master on, but bus voltage stays low: battery flat?"});
    v.push_back({"Avionics master OFF for start", set("electrical.avionics", 0), nullptr, 5, ""});
    v.push_back({"Fuel shutoff valve ON", set("fuel.shutoff", 1), nullptr, 5, ""});
    v.push_back({"Fuel selector BOTH", set("fuel.selector", both), nullptr, 5, ""});
    v.push_back({"Ignition BOTH", set("engine1.ignition", 3), nullptr, 5, ""});
    v.push_back({"Mixture RICH, throttle 1/4 in, crank", [](Aircraft&, PilotInput& in) {
                     in.mixture = 1.0;
                     in.throttle = 0.12;
                     in.starterHeld = true;
                 },
                 [](const Aircraft& a, Real) { return a.telemetry().engines[0].engine.running && a.telemetry().engines[0].rpm > 600; },
                 12, "Engine did not start within 12 s of cranking (fuel, battery or ignition?)"});
    v.push_back({"Release starter, set 1000 rpm", [](Aircraft&, PilotInput& in) {
                     in.starterHeld = false;
                     in.mixture = 1.0;
                     in.throttle = 0.12;
                 },
                 [](const Aircraft&, Real t) { return t > 1.0; }, 5, ""});
    v.push_back({"Alternator ON", set("electrical.alternator", 1),
                 [](const Aircraft& a, Real t) { return t > 0.5 && a.electrical().alternatorOnline(); }, 5,
                 "Alternator did not come online"});
    v.push_back({"Avionics master ON", set("electrical.avionics", 1), nullptr, 5, ""});
    return v;
}

}  // namespace avd
