// Ballistic Dynamics — the data library: loading, units, provenance.
// SPDX-License-Identifier: MIT
#include "bd/library.hpp"
#include "bd/units.hpp"
#include "bd_test.hpp"

using namespace bd;

namespace {
bool mentions(const std::vector<std::string>& errors, const char* text) {
    for (const auto& e : errors)
        if (e.find(text) != std::string::npos) return true;
    return false;
}
}  // namespace

TEST_CASE("data: the shipped library loads, resolves and is in SI") {
    const Library& lib = bdtest::library();
    const ProjectileDef* p = lib.projectile("smk_308_175");
    CHECK(p != nullptr);
    if (!p) return;
    CHECK_NEAR(p->mass, units::grains(175), 1e-15);
    CHECK_NEAR(p->diameter, 0.0078232, 1e-12);
    CHECK(p->bcG1.values.size() == 3);
    CHECK_NEAR(p->bcG1.values[0], units::bcToSi(0.505), 1e-12);
    CHECK_NEAR(p->bcG1.minSpeeds[0], units::fps(2800), 1e-12);
    CHECK(p->provenance.at("bc_g1").kind == ProvenanceKind::Published);
    CHECK(p->provenance.at("length_in").kind == ProvenanceKind::Estimated);
    const CartridgeDef* c = lib.cartridge("fgmm_308_175");
    CHECK(c && c->projectile == "smk_308_175");
    CHECK(c && std::abs(c->muzzleVelocity - 792.48) < 1e-9);
    const RifleDef* r = lib.rifle("m24_sws");
    CHECK(r && r->rightHandTwist);
    CHECK(r && std::abs(r->twist - 11.25 * 0.0254) < 1e-12);
}

TEST_CASE("data: every number must say where it came from") {
    Library lib;
    // No note.
    auto e = lib.load(R"({"rifles": [{"id": "a", "name": "A", "twist_in": 10, "barrel_in": 20,
        "provenance": {"twist_in": "published: someone"}}]})");
    CHECK(mentions(e, "\"barrel_in\" has no provenance note"));
    // A note without its kind, or a kind with nothing behind it.
    e = lib.load(R"({"rifles": [{"id": "b", "name": "B", "twist_in": 10, "barrel_in": 20,
        "provenance": {"twist_in": "from the box", "barrel_in": "measured:"}}]})");
    CHECK(mentions(e, "provenance of \"twist_in\" must start with"));
    CHECK(mentions(e, "provenance of \"barrel_in\" must start with"));
    // A stale note for a field that is not there.
    e = lib.load(R"({"rifles": [{"id": "c", "name": "C", "twist_in": 10, "barrel_in": 20,
        "provenance": {"twist_in": "measured: a cleaning rod", "barrel_in": "measured: tape", "weight_lb": "estimated: x"}}]})");
    CHECK(mentions(e, "missing field \"weight_lb\""));
    // Nested objects carry their own notes.
    e = lib.load(R"({"rifles": [{"id": "d", "name": "D", "twist_in": 10, "barrel_in": 20,
        "scope": {"height_in": 1.5},
        "provenance": {"twist_in": "measured: rod", "barrel_in": "measured: tape"}}]})");
    CHECK(mentions(e, "rifles[0] \"d\".scope: \"height_in\" has no provenance note"));
    // None of the bad records was kept.
    CHECK(lib.rifles().empty());
    // A good one is.
    e = lib.load(R"({"rifles": [{"id": "ok", "name": "OK", "twist_in": 8, "barrel_in": 26, "twist_direction": "left",
        "provenance": {"twist_in": "measured: cleaning rod, one turn", "barrel_in": "measured: bolt face to muzzle"}}]})");
    CHECK(e.empty());
    CHECK(lib.rifle("ok") && !lib.rifle("ok")->rightHandTwist);
    CHECK(lib.rifle("ok") && lib.rifle("ok")->provenance.at("twist_in").kind == ProvenanceKind::Measured);
}

TEST_CASE("data: shapes, ranges and cross-references are checked") {
    Library lib;
    auto e = lib.load(R"({"drag_curves": [{"id": "x", "name": "X", "family": "G5", "mach": [0, 0.5, 0.4], "cd": [0.2, 0.2],
        "provenance": {"mach": "estimated: t", "cd": "estimated: t"}}]})");
    CHECK(mentions(e, "family"));
    CHECK(mentions(e, "same length"));
    CHECK(mentions(e, "strictly ascending"));
    e = lib.load(R"({"projectiles": [{"id": "p", "name": "P", "diameter_in": 0.308, "mass_gr": -1, "length_in": 1.2,
        "provenance": {"diameter_in": "published: x", "mass_gr": "published: x", "length_in": "published: x"}}]})");
    CHECK(mentions(e, "\"mass_gr\" must be positive"));
    CHECK(mentions(e, "needs a ballistic coefficient"));
    e = lib.load(R"({"projectiles": [{"id": "s", "name": "S", "diameter_in": 0.308, "mass_gr": 150, "length_in": 1.1,
        "bc_g1": [0.4, 0.42], "bc_g1_min_fps": [1000, 2000],
        "provenance": {"diameter_in": "published: x", "mass_gr": "published: x", "length_in": "published: x",
                       "bc_g1": "published: x", "bc_g1_min_fps": "published: x"}}]})");
    CHECK(mentions(e, "strictly descending"));
    CHECK(mentions(e, "must end at 0"));
    e = lib.load(R"({"cartridges": [{"id": "k", "name": "K", "projectile": "nope", "muzzle_velocity_fps": 2700, "test_barrel_in": 24,
        "provenance": {"muzzle_velocity_fps": "published: x", "test_barrel_in": "published: x"}}]})");
    CHECK(e.empty());
    CHECK(mentions(lib.resolve(), "unknown projectile \"nope\""));
    CHECK(mentions(lib.load("{\"rifles\": 3}"), "must be an array"));
    CHECK(mentions(lib.load("{\"guns\": []}"), "unknown section"));
    CHECK(mentions(lib.load("{\"rifles\": [}"), "JSON: line 1"));
    // Duplicate ids.
    const char* k = R"({"cartridges": [{"id": "k", "name": "K", "projectile": "nope", "muzzle_velocity_fps": 2700, "test_barrel_in": 24,
        "provenance": {"muzzle_velocity_fps": "published: x", "test_barrel_in": "published: x"}}]})";
    CHECK(mentions(lib.load(k), "duplicate id"));
}
