// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// JSON -> AircraftDefinition (schema v1, docs/architecture/aircraft-definition-schema.md).
// Units are explicit in key suffixes (_m, _kg, _deg, _l, _kw ...); the loader
// converts to SI. Strict: unknown keys and wrong types are errors with their
// JSON path, and the result is then run through AircraftDefinition::validate().
#pragma once

#include <string>
#include <vector>

#include "avd/aircraft_def.hpp"

namespace avd {

struct LoadResult {
    AircraftDefinition definition;
    std::vector<std::string> errors;  // parse, schema and validation errors
    bool ok() const { return errors.empty(); }
};

LoadResult loadAircraftFromJson(const std::string& jsonText);
LoadResult loadAircraftFromFile(const std::string& path);

}  // namespace avd
