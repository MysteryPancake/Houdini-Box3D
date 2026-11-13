// SIM_Box3DParms.h
//
// Builds a SIM data node's parameter templates from a table: one call per parameter (name,
// label, default, range, help, optional disable condition) and one per tab, so the PRM
// boilerplate, the tab counts and the storage the templates point at are handled in one place.
// Keep the builder alive for the life of the plugin (a function-local static): Houdini keeps
// pointers into it.
#pragma once

#include <PRM/PRM_Conditional.h>   // PRM_Template.h only forward-declares PRM_Conditional
#include <PRM/PRM_Include.h>

#include <array>
#include <deque>
#include <initializer_list>
#include <string>
#include <vector>

class Box3DParms
{
public:
    // A slider range: hard minimum `min`, slider maximum `max` (hard too with `hardMax`).
    struct Range
    {
        double min = 0.0, max = 1.0;
        bool   hardMax = false;
    };

    // Starts a tab; parameters added after it go on it. No tabs: a flat list.
    Box3DParms &tab(const char *label)
    {
        myTabs.push_back({ label, 0 });
        return *this;
    }

    Box3DParms &toggle(const char *name, const char *label, bool def, const char *help,
                       const char *disableWhen = nullptr)
    {
        return add(PRM_TOGGLE, name, label, { def ? 1.0 : 0.0 }, 1, nullptr, nullptr, help, disableWhen);
    }

    Box3DParms &number(const char *name, const char *label, double def, const char *help,
                       const char *disableWhen = nullptr)
    {
        return add(PRM_FLT_J, name, label, { def }, 1, nullptr, nullptr, help, disableWhen);
    }

    Box3DParms &number(const char *name, const char *label, double def, Range r, const char *help,
                       const char *disableWhen = nullptr)
    {
        return add(PRM_FLT_J, name, label, { def }, 1, range(r), nullptr, help, disableWhen);
    }

    Box3DParms &integer(const char *name, const char *label, int def, Range r, const char *help,
                        const char *disableWhen = nullptr)
    {
        return add(PRM_INT_J, name, label, { double(def) }, 1, range(r), nullptr, help, disableWhen);
    }

    // Three floats (PRM_XYZ_J lays them out as x / y / z).
    Box3DParms &vector(const char *name, const char *label, std::array<double, 3> def, const char *help,
                       PRM_Type type = PRM_FLT_J, const char *disableWhen = nullptr)
    {
        return add(type, name, label, { def[0], def[1], def[2] }, 3, nullptr, nullptr, help, disableWhen);
    }

    // A menu whose tokens are "0", "1", ... in the order of `items` (an int option).
    Box3DParms &menu(const char *name, const char *label, int def, std::initializer_list<const char *> items,
                     const char *help)
    {
        myChoices.emplace_back();
        std::vector<PRM_Name> &choices = myChoices.back();
        int i = 0;
        for (const char *item : items)
        {
            myStrings.push_back(std::to_string(i++));
            choices.emplace_back(myStrings.back().c_str(), item);
        }
        choices.emplace_back();          // terminator
        myMenus.emplace_back(PRM_CHOICELIST_SINGLE, choices.data());
        return add(PRM_ORD, name, label, { double(def) }, 1, nullptr, &myMenus.back(), help, nullptr);
    }

    // The finished list (switcher first if there are tabs), terminated.
    const PRM_Template *templates()
    {
        if (myBuilt.empty())
        {
            if (!myTabs.empty())
            {
                for (const Tab &t : myTabs)
                    myTabDefaults.emplace_back(fpreal(t.count), t.label);
                myBuilt.push_back(PRM_Template(PRM_SWITCHER, int(myTabDefaults.size()), &myTabsName,
                                               myTabDefaults.data()));
            }
            myBuilt.insert(myBuilt.end(), myParms.begin(), myParms.end());
            myBuilt.emplace_back();
        }
        return myBuilt.data();
    }

private:
    struct Tab
    {
        const char *label;
        int         count;
    };

    PRM_Range *range(Range r)
    {
        myRanges.emplace_back(PRM_RANGE_RESTRICTED, r.min, r.hardMax ? PRM_RANGE_RESTRICTED : PRM_RANGE_UI, r.max);
        return &myRanges.back();
    }

    Box3DParms &add(PRM_Type type, const char *name, const char *label, std::initializer_list<double> defs,
                    int size, PRM_Range *r, PRM_ChoiceList *menu, const char *help, const char *disableWhen)
    {
        myNames.emplace_back(name, label);
        myDefaults.emplace_back();
        std::array<PRM_Default, 3> &d = myDefaults.back();
        size_t i = 0;
        for (double v : defs)
            d[i++] = PRM_Default(v);
        // disableWhen: a Houdini disable-when string ("{ toggle == 0 }"), copied so callers can
        // pass a temporary. Null or empty: always enabled.
        PRM_Conditional *cond = nullptr;
        if (disableWhen && *disableWhen)
        {
            myStrings.emplace_back(disableWhen);
            myConditions.emplace_back(myStrings.back().c_str(), PRM_CONDTYPE_DISABLE);
            cond = &myConditions.back();
        }
        myParms.push_back(PRM_Template(type, size, &myNames.back(), d.data(), menu, r, 0, 0, 1, help, cond));
        if (!myTabs.empty())
            ++myTabs.back().count;
        return *this;
    }

    // deques: stable addresses as they grow (the templates point into them).
    std::deque<std::string>                myStrings;
    std::deque<PRM_Name>                   myNames;
    std::deque<std::array<PRM_Default, 3>> myDefaults;
    std::deque<PRM_Range>                  myRanges;
    std::deque<std::vector<PRM_Name>>      myChoices;
    std::deque<PRM_ChoiceList>             myMenus;
    std::deque<PRM_Conditional>            myConditions;
    std::vector<Tab>                       myTabs;
    std::vector<PRM_Default>               myTabDefaults;
    PRM_Name                               myTabsName{ "b3dtabs" };
    std::vector<PRM_Template>              myParms;
    std::vector<PRM_Template>              myBuilt;
};
