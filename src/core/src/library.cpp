// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/library.hpp"

#include "bd/units.hpp"

namespace bd {

bool parseProvenance(const std::string& text, Provenance& out) {
    struct Tag {
        const char* prefix;
        ProvenanceKind kind;
    };
    static const Tag kTags[] = {{"published:", ProvenanceKind::Published},
                                {"measured:", ProvenanceKind::Measured},
                                {"estimated:", ProvenanceKind::Estimated}};
    for (const Tag& t : kTags) {
        const std::string p = t.prefix;
        if (text.compare(0, p.size(), p) == 0) {
            std::string note = text.substr(p.size());
            while (!note.empty() && note.front() == ' ') note.erase(note.begin());
            if (note.empty()) return false;  // a tag with nothing behind it says nothing
            out.kind = t.kind;
            out.note = std::move(note);
            return true;
        }
    }
    return false;
}

namespace {

bool holdsNumbers(const json::Value& v) {
    if (v.isNumber()) return true;
    if (v.isArray())
        for (const auto& e : v.items())
            if (e.isNumber() || (e.isArray() && holdsNumbers(e))) return true;
    return false;
}

void checkObject(const json::Value& obj, const std::string& where, std::vector<std::string>& errors,
                 ProvenanceMap* out) {
    const json::Value* prov = obj.find("provenance");
    if (prov && !prov->isObject()) {
        errors.push_back(where + ": \"provenance\" must be an object");
        prov = nullptr;
    }
    for (const auto& [key, v] : obj.members()) {
        if (key == "provenance") continue;
        if (v.isObject()) {
            checkObject(v, where + "." + key, errors, nullptr);
            continue;
        }
        if (v.isArray())
            for (std::size_t i = 0; i < v.items().size(); ++i)
                if (v.items()[i].isObject())
                    checkObject(v.items()[i], where + "." + key + "[" + std::to_string(i) + "]", errors, nullptr);
        if (!holdsNumbers(v)) continue;
        const json::Value* note = prov ? prov->find(key) : nullptr;
        if (!note || !note->isString()) {
            errors.push_back(where + ": \"" + key + "\" has no provenance note");
            continue;
        }
        Provenance p;
        if (!parseProvenance(note->asString(), p)) {
            errors.push_back(where + ": provenance of \"" + key +
                             "\" must start with published:, measured: or estimated: and say more");
            continue;
        }
        if (out) (*out)[key] = p;
    }
    // A note for a field that does not exist is a stale note.
    if (prov)
        for (const auto& [key, v] : prov->members()) {
            (void)v;
            if (!obj.find(key)) errors.push_back(where + ": provenance note for missing field \"" + key + "\"");
        }
}

struct Reader {
    const json::Value& obj;
    std::string where;
    std::vector<std::string>& errors;
    bool ok = true;

    std::string str(const char* key, bool required = true) {
        const json::Value* v = obj.find(key);
        if (!v) {
            if (required) bad(std::string("missing \"") + key + "\"");
            return {};
        }
        if (!v->isString()) {
            bad(std::string("\"") + key + "\" must be a string");
            return {};
        }
        return v->asString();
    }
    // A positive number times `scale`; 0 when absent and not required.
    Real num(const char* key, Real scale, bool required = true, bool allowZero = false) {
        const json::Value* v = obj.find(key);
        if (!v) {
            if (required) bad(std::string("missing \"") + key + "\"");
            return 0;
        }
        if (!v->isNumber()) {
            bad(std::string("\"") + key + "\" must be a number");
            return 0;
        }
        const Real d = v->asNumber();
        if (d < 0 || (d == 0 && !allowZero)) {
            bad(std::string("\"") + key + "\" must be positive");
            return 0;
        }
        return d * scale;
    }
    std::vector<Real> nums(const char* key, Real scale, bool required = true) {
        std::vector<Real> out;
        const json::Value* v = obj.find(key);
        if (!v) {
            if (required) bad(std::string("missing \"") + key + "\"");
            return out;
        }
        if (!v->isArray()) {
            bad(std::string("\"") + key + "\" must be an array of numbers");
            return out;
        }
        for (const auto& e : v->items()) {
            if (!e.isNumber()) {
                bad(std::string("\"") + key + "\" must be an array of numbers");
                return {};
            }
            out.push_back(e.asNumber() * scale);
        }
        return out;
    }
    void bad(const std::string& msg) {
        errors.push_back(where + ": " + msg);
        ok = false;
    }
};

BallisticCoefficient readBc(Reader& r, const char* family, const char* valuesKey, const char* speedsKey) {
    BallisticCoefficient bc;
    bc.family = family;
    if (!r.obj.find(valuesKey)) return bc;
    bc.values = r.nums(valuesKey, units::kBcLbPerSqInToSi);
    if (r.obj.find(speedsKey)) {
        bc.minSpeeds = r.nums(speedsKey, units::kFoot);
    } else if (bc.values.size() == 1) {
        bc.minSpeeds.assign(1, 0.0);
    }
    if (bc.values.size() != bc.minSpeeds.size()) {
        r.bad(std::string("\"") + valuesKey + "\" and \"" + speedsKey + "\" must have the same length");
        return {};
    }
    for (std::size_t i = 0; i < bc.values.size(); ++i) {
        if (!(bc.values[i] > 0)) r.bad(std::string("\"") + valuesKey + "\" values must be positive");
        if (i > 0 && !(bc.minSpeeds[i] < bc.minSpeeds[i - 1]))
            r.bad(std::string("\"") + speedsKey + "\" must be strictly descending");
    }
    if (!bc.minSpeeds.empty() && bc.minSpeeds.back() != 0)
        r.bad(std::string("\"") + speedsKey + "\" must end at 0 so every speed is covered");
    return bc;
}

template <class T>
const T* byId(const std::vector<T>& v, const std::string& id) {
    for (const auto& e : v)
        if (e.id == id) return &e;
    return nullptr;
}

template <class T>
void insert(std::vector<T>& v, T&& rec, const std::string& where, std::vector<std::string>& errors) {
    for (const auto& e : v)
        if (e.id == rec.id) {
            errors.push_back(where + ": duplicate id \"" + rec.id + "\"");
            return;
        }
    v.push_back(std::move(rec));
}

}  // namespace

void checkProvenance(const json::Value& record, const std::string& where, std::vector<std::string>& errors,
                     ProvenanceMap* out) {
    checkObject(record, where, errors, out);
}

std::vector<std::string> Library::load(const std::string& jsonText) {
    std::vector<std::string> errors;
    const json::ParseResult pr = json::parse(jsonText);
    if (!pr.ok()) {
        errors.push_back("JSON: " + pr.error);
        return errors;
    }
    const json::Value& root = pr.value;
    if (!root.isObject()) {
        errors.push_back("JSON: the document must be an object");
        return errors;
    }
    for (const auto& [section, list] : root.members()) {
        if (section == "comment" || section == "schema") continue;
        if (section != "drag_curves" && section != "projectiles" && section != "cartridges" && section != "rifles") {
            errors.push_back("unknown section \"" + section + "\"");
            continue;
        }
        if (!list.isArray()) {
            errors.push_back("\"" + section + "\" must be an array");
            continue;
        }
        for (std::size_t i = 0; i < list.items().size(); ++i) {
            const json::Value& rec = list.items()[i];
            std::string where = section + "[" + std::to_string(i) + "]";
            if (!rec.isObject()) {
                errors.push_back(where + ": must be an object");
                continue;
            }
            if (const json::Value* id = rec.find("id"); id && id->isString()) where += " \"" + id->asString() + "\"";
            const std::size_t before = errors.size();
            ProvenanceMap prov;
            checkProvenance(rec, where, errors, &prov);
            Reader r{rec, where, errors};
            const std::string id = r.str("id");
            const std::string name = r.str("name");
            if (section == "drag_curves") {
                DragCurveDef d;
                d.id = id;
                d.name = name;
                d.family = r.str("family");
                if (d.family != "G1" && d.family != "G7" && d.family != "bullet")
                    r.bad("\"family\" must be G1, G7 or bullet");
                d.mach = r.nums("mach", 1.0);
                d.cd = r.nums("cd", 1.0);
                if (d.mach.size() != d.cd.size() || d.mach.size() < 2)
                    r.bad("\"mach\" and \"cd\" must have the same length, at least 2");
                for (std::size_t k = 0; k < d.mach.size(); ++k) {
                    if (k > 0 && !(d.mach[k] > d.mach[k - 1])) r.bad("\"mach\" must be strictly ascending");
                    if (k < d.cd.size() && !(d.cd[k] > 0)) r.bad("\"cd\" must be positive");
                }
                if (!d.mach.empty() && d.mach.front() < 0) r.bad("\"mach\" must not be negative");
                d.provenance = std::move(prov);
                if (r.ok && errors.size() == before) insert(drag_, std::move(d), where, errors);
            } else if (section == "projectiles") {
                ProjectileDef p;
                p.id = id;
                p.name = name;
                p.construction = r.str("construction", false);
                p.diameter = r.num("diameter_in", units::kInch);
                p.mass = r.num("mass_gr", units::kGrain);
                p.length = r.num("length_in", units::kInch);
                p.bcG1 = readBc(r, "G1", "bc_g1", "bc_g1_min_fps");
                p.bcG7 = readBc(r, "G7", "bc_g7", "bc_g7_min_fps");
                p.dragCurve = r.str("drag_curve", false);
                if (p.bcG1.empty() && p.bcG7.empty() && p.dragCurve.empty())
                    r.bad("needs a ballistic coefficient (bc_g1 or bc_g7) or a drag_curve");
                if (p.length > 0 && p.diameter > 0 && p.length < p.diameter)
                    r.bad("\"length_in\" is shorter than the diameter");
                p.provenance = std::move(prov);
                if (r.ok && errors.size() == before) insert(projectiles_, std::move(p), where, errors);
            } else if (section == "cartridges") {
                CartridgeDef c;
                c.id = id;
                c.name = name;
                c.projectile = r.str("projectile");
                c.muzzleVelocity = r.num("muzzle_velocity_fps", units::kFoot);
                c.testBarrel = r.num("test_barrel_in", units::kInch);
                c.provenance = std::move(prov);
                if (r.ok && errors.size() == before) insert(cartridges_, std::move(c), where, errors);
            } else {
                RifleDef f;
                f.id = id;
                f.name = name;
                f.twist = r.num("twist_in", units::kInch);
                const std::string dir = r.str("twist_direction", false);
                if (!dir.empty() && dir != "right" && dir != "left") r.bad("\"twist_direction\" must be right or left");
                f.rightHandTwist = dir != "left";
                f.barrel = r.num("barrel_in", units::kInch);
                f.sightHeight = r.num("sight_height_in", units::kInch, false, true);
                f.provenance = std::move(prov);
                if (r.ok && errors.size() == before) insert(rifles_, std::move(f), where, errors);
            }
        }
    }
    return errors;
}

std::vector<std::string> Library::resolve() const {
    std::vector<std::string> errors;
    for (const auto& c : cartridges_)
        if (!projectile(c.projectile))
            errors.push_back("cartridge \"" + c.id + "\": unknown projectile \"" + c.projectile + "\"");
    for (const auto& p : projectiles_) {
        if (p.dragCurve.empty()) continue;
        const DragCurveDef* d = dragCurve(p.dragCurve);
        if (!d)
            errors.push_back("projectile \"" + p.id + "\": unknown drag curve \"" + p.dragCurve + "\"");
        else if (d->family != "bullet")
            errors.push_back("projectile \"" + p.id + "\": drag_curve must be a bullet curve; standard tables go with a BC");
    }
    return errors;
}

const DragCurveDef* Library::dragCurve(const std::string& id) const { return byId(drag_, id); }
const ProjectileDef* Library::projectile(const std::string& id) const { return byId(projectiles_, id); }
const CartridgeDef* Library::cartridge(const std::string& id) const { return byId(cartridges_, id); }
const RifleDef* Library::rifle(const std::string& id) const { return byId(rifles_, id); }

}  // namespace bd
