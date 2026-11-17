// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// The data library: drag curves, projectiles, cartridges and rifles, loaded
// from JSON. Every number carries a provenance note (docs/data-format.md):
//
//   "provenance": { "mass_gr": "published: Sierra catalogue #2275", ... }
//
// The note starts with its kind, `published:` (with its source), `measured:`
// (what, how, by whom) or `estimated:` (with the reasoning). The loader
// rejects a record with a number whose provenance is missing or untagged,
// at any depth: every object holding numbers carries its own "provenance".
//
// Units are in the field names (`_gr`, `_in`, `_fps`) and converted to SI
// here; the structs below hold SI only.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "bd/json.hpp"
#include "bd/math.hpp"

namespace bd {

enum class ProvenanceKind { Published, Measured, Estimated };

struct Provenance {
    ProvenanceKind kind = ProvenanceKind::Estimated;
    std::string note;  // the text after the tag
};

// Field name (as in the JSON) -> its provenance.
using ProvenanceMap = std::map<std::string, Provenance>;

// Drag coefficient against Mach number. `family` G1 or G7 marks a standard
// reference projectile (scaled by a ballistic coefficient); `bullet` is a
// measured curve for one projectile (used as is, with its own reference area).
struct DragCurveDef {
    std::string id, name, family;
    std::vector<Real> mach, cd;
    ProvenanceMap provenance;
};

// A ballistic coefficient published against one standard, possibly stepped by
// velocity (Sierra style): `values[i]` applies at and above `minSpeeds[i]`.
struct BallisticCoefficient {
    std::string family;          // "G1" or "G7"
    std::vector<Real> values;    // kg/m^2 (SI)
    std::vector<Real> minSpeeds; // m/s, descending
    bool empty() const { return values.empty(); }
};

struct ProjectileDef {
    std::string id, name, construction;
    Real diameter = 0;  // m
    Real mass = 0;      // kg
    Real length = 0;    // m
    BallisticCoefficient bcG1, bcG7;
    std::string dragCurve;  // optional id of a bullet-specific curve
    ProvenanceMap provenance;
};

struct CartridgeDef {
    std::string id, name, projectile;
    Real muzzleVelocity = 0;  // m/s, as published
    Real testBarrel = 0;      // m, the barrel length that velocity was measured in
    ProvenanceMap provenance;
};

struct RifleDef {
    std::string id, name;
    Real twist = 0;          // m per turn
    bool rightHandTwist = true;
    Real barrel = 0;         // m
    Real sightHeight = 0;    // m, bore axis to the optic's axis
    ProvenanceMap provenance;
};

class Library {
public:
    // Parses one JSON document holding any of "drag_curves", "projectiles",
    // "cartridges", "rifles". Returns the errors (empty when fine); records
    // with errors are skipped, the rest are kept.
    std::vector<std::string> load(const std::string& jsonText);
    // Cross-references (a cartridge's projectile, a projectile's drag curve).
    std::vector<std::string> resolve() const;

    const DragCurveDef* dragCurve(const std::string& id) const;
    const ProjectileDef* projectile(const std::string& id) const;
    const CartridgeDef* cartridge(const std::string& id) const;
    const RifleDef* rifle(const std::string& id) const;

    const std::vector<DragCurveDef>& dragCurves() const { return drag_; }
    const std::vector<ProjectileDef>& projectiles() const { return projectiles_; }
    const std::vector<CartridgeDef>& cartridges() const { return cartridges_; }
    const std::vector<RifleDef>& rifles() const { return rifles_; }

private:
    std::vector<DragCurveDef> drag_;
    std::vector<ProjectileDef> projectiles_;
    std::vector<CartridgeDef> cartridges_;
    std::vector<RifleDef> rifles_;
};

// Checks that every number (or array of numbers) in `record`, at any depth, has
// a tagged provenance note in its object's "provenance". Appends errors
// prefixed by `where`; fills `out` with the record's top-level notes.
void checkProvenance(const json::Value& record, const std::string& where, std::vector<std::string>& errors,
                     ProvenanceMap* out = nullptr);
// Parses "published: ..." etc. False when the tag is missing.
bool parseProvenance(const std::string& text, Provenance& out);

}  // namespace bd
