// SIM_Box3DParms.h
//
// Builds a SIM data node's parameter templates from a table: one call per
// parameter (name, label, default, range, help, optional disable condition) and
// one per tab or folder, so the PRM boilerplate, the counts and the storage the
// templates point at are handled in one place. Groups are either tabs (one
// switcher) or simple folders (a one-folder switcher each, with
// PRM_SpareData::groupTypeSimple); a builder uses one kind only, except that a
// tab can hold simple folders (subfolder()). Keep the
// builder alive for the life of the plugin (a function-local static): Houdini
// keeps pointers into it.
#pragma once

#include <PRM/PRM_Conditional.h> // PRM_Template.h only forward-declares PRM_Conditional
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>

#include <array>
#include <deque>
#include <initializer_list>
#include <string>
#include <vector>

class Box3DParms {
public:
  // A slider range: minimum `min` (hard unless `hardMin` is false), slider
  // maximum `max` (hard too with `hardMax`).
  struct Range {
    double min = 0.0, max = 1.0;
    bool hardMax = false;
    bool hardMin = true;
  };

  // folderPrefix names the simple folders' switchers (<prefix>0, <prefix>1,
  // ...); tabs share one switcher, "b3dtabs".
  explicit Box3DParms(const char *folderPrefix = "b3dfolder")
      : myFolderPrefix(folderPrefix) {}

  // Starts a tab; parameters added after it go on it. No groups: a flat list.
  Box3DParms &tab(const char *label) {
    myGroups.push_back({label, myParms.size(), 0});
    mySimple = false;
    myInSub = false;
    return *this;
  }

  // Starts a simple folder inside the current tab (tabs only); parameters added
  // after it go in it, until the next subfolder() or tab(). In the tab's count
  // the folder's switcher is one parameter, its contents none.
  Box3DParms &subfolder(const char *label) {
    mySubs.push_back({label, myParms.size(), 0});
    myInSub = true;
    if (!myGroups.empty())
      ++myGroups.back().count;
    return *this;
  }

  // Starts a simple folder instead (don't mix with tab()).
  Box3DParms &folder(const char *label) {
    myGroups.push_back({label, myParms.size(), 0});
    mySimple = true;
    return *this;
  }

  Box3DParms &toggle(const char *name, const char *label, bool def,
                     const char *help, const char *disableWhen = nullptr) {
    return add(PRM_TOGGLE, name, label, {def ? 1.0 : 0.0}, 1, nullptr, nullptr,
               help, disableWhen);
  }

  Box3DParms &number(const char *name, const char *label, double def,
                     const char *help, const char *disableWhen = nullptr) {
    return add(PRM_FLT_J, name, label, {def}, 1, nullptr, nullptr, help,
               disableWhen);
  }

  Box3DParms &number(const char *name, const char *label, double def, Range r,
                     const char *help, const char *disableWhen = nullptr) {
    return add(PRM_FLT_J, name, label, {def}, 1, range(r), nullptr, help,
               disableWhen);
  }

  Box3DParms &integer(const char *name, const char *label, int def, Range r,
                      const char *help, const char *disableWhen = nullptr) {
    return add(PRM_INT_J, name, label, {static_cast<double>(def)}, 1, range(r),
               nullptr, help, disableWhen);
  }

  // Three floats (PRM_XYZ_J lays them out as x / y / z).
  Box3DParms &vector(const char *name, const char *label,
                     std::array<double, 3> def, const char *help,
                     PRM_Type type = PRM_FLT_J,
                     const char *disableWhen = nullptr) {
    return add(type, name, label, {def[0], def[1], def[2]}, 3, nullptr, nullptr,
               help, disableWhen);
  }

  // A menu whose tokens are "0", "1", ... in the order of `items` (an int
  // option).
  Box3DParms &menu(const char *name, const char *label, int def,
                   std::initializer_list<const char *> items,
                   const char *help) {
    myChoices.emplace_back();
    std::vector<PRM_Name> &choices = myChoices.back();
    int i = 0;
    for (const char *item : items) {
      myStrings.push_back(std::to_string(i++));
      choices.emplace_back(myStrings.back().c_str(), item);
    }
    choices.emplace_back(); // terminator
    myMenus.emplace_back(PRM_CHOICELIST_SINGLE, choices.data());
    return add(PRM_ORD, name, label, {static_cast<double>(def)}, 1, nullptr,
               &myMenus.back(), help, nullptr);
  }

  // The finished list, terminated: tabs' switcher first, or each simple
  // folder's switcher before its parameters.
  const PRM_Template *templates() {
    if (!myBuilt.empty())
      return myBuilt.data();
    if (!mySimple) {
      if (!myGroups.empty()) {
        for (const Group &g : myGroups)
          myTabDefaults.emplace_back(static_cast<fpreal>(g.count), g.label);
        myBuilt.push_back(PRM_Template(PRM_SWITCHER,
                                       static_cast<int>(myTabDefaults.size()),
                                       &myTabsName, myTabDefaults.data()));
      }
      // Each subfolder's switcher goes just before its first parameter.
      size_t sub = 0;
      for (size_t i = 0; i <= myParms.size(); ++i) {
        while (sub < mySubs.size() && mySubs[sub].start == i)
          addSubfolderSwitcher(sub++);
        if (i < myParms.size())
          myBuilt.push_back(myParms[i]);
      }
    } else {
      size_t next = 0; // first parameter not yet placed
      for (size_t i = 0; i < myGroups.size(); ++i) {
        const Group &g = myGroups[i];
        // Parameters added before the first folder stay above it.
        myBuilt.insert(myBuilt.end(), myParms.begin() + next,
                       myParms.begin() + g.start);
        myStrings.push_back(myFolderPrefix + std::to_string(i));
        myNames.emplace_back(myStrings.back().c_str());
        myFolderDefaults.emplace_back(static_cast<fpreal>(g.count), g.label);
        myBuilt.push_back(PRM_Template(
            PRM_SWITCHER, 1, &myNames.back(), &myFolderDefaults.back(), nullptr,
            nullptr, nullptr, &PRM_SpareData::groupTypeSimple));
        myBuilt.insert(myBuilt.end(), myParms.begin() + g.start,
                       myParms.begin() + g.start + g.count);
        next = g.start + static_cast<size_t>(g.count);
      }
      myBuilt.insert(myBuilt.end(), myParms.begin() + next, myParms.end());
    }
    myBuilt.emplace_back();
    return myBuilt.data();
  }

private:
  struct Group {
    const char *label;
    size_t start; // index in myParms of its first parameter
    int count;
  };

  void addSubfolderSwitcher(size_t i) {
    const Group &g = mySubs[i];
    myStrings.push_back(myFolderPrefix + "sub" + std::to_string(i));
    myNames.emplace_back(myStrings.back().c_str());
    myFolderDefaults.emplace_back(static_cast<fpreal>(g.count), g.label);
    myBuilt.push_back(PRM_Template(PRM_SWITCHER, 1, &myNames.back(),
                                   &myFolderDefaults.back(), nullptr, nullptr,
                                   nullptr, &PRM_SpareData::groupTypeSimple));
  }

  PRM_Range *range(Range r) {
    myRanges.emplace_back(
        r.hardMin ? PRM_RANGE_RESTRICTED : PRM_RANGE_UI, r.min,
        r.hardMax ? PRM_RANGE_RESTRICTED : PRM_RANGE_UI, r.max);
    return &myRanges.back();
  }

  Box3DParms &add(PRM_Type type, const char *name, const char *label,
                  std::initializer_list<double> defs, int size, PRM_Range *r,
                  PRM_ChoiceList *menu, const char *help,
                  const char *disableWhen) {
    myNames.emplace_back(name, label);
    myDefaults.emplace_back();
    std::array<PRM_Default, 3> &d = myDefaults.back();
    size_t i = 0;
    for (double v : defs)
      d[i++] = PRM_Default(v);
    // disableWhen: a Houdini disable-when string ("{ toggle == 0 }"), copied so
    // callers can pass a temporary. Null or empty: always enabled.
    PRM_Conditional *cond = nullptr;
    if (disableWhen && *disableWhen) {
      myStrings.emplace_back(disableWhen);
      myConditions.emplace_back(myStrings.back().c_str(), PRM_CONDTYPE_DISABLE);
      cond = &myConditions.back();
    }
    myParms.push_back(PRM_Template(type, size, &myNames.back(), d.data(), menu,
                                   r, 0, 0, 1, help, cond));
    if (myInSub)
      ++mySubs.back().count;
    else if (!myGroups.empty())
      ++myGroups.back().count;
    return *this;
  }

  // deques: stable addresses as they grow (the templates point into them).
  std::deque<std::string> myStrings;
  std::deque<PRM_Name> myNames;
  std::deque<std::array<PRM_Default, 3>> myDefaults;
  std::deque<PRM_Range> myRanges;
  std::deque<std::vector<PRM_Name>> myChoices;
  std::deque<PRM_ChoiceList> myMenus;
  std::deque<PRM_Conditional> myConditions;
  std::vector<Group> myGroups;
  std::vector<Group> mySubs; // simple folders inside tabs (subfolder())
  bool mySimple = false;
  bool myInSub = false;
  // Tabs: one array, filled before the switcher points at it. Folders: one
  // each, in a deque so earlier ones keep their addresses.
  std::vector<PRM_Default> myTabDefaults;
  std::deque<PRM_Default> myFolderDefaults;
  std::string myFolderPrefix;
  PRM_Name myTabsName{"b3dtabs"};
  std::vector<PRM_Template> myParms;
  std::vector<PRM_Template> myBuilt;
};
