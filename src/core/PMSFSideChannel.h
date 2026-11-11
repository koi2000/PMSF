#ifndef PMSF_CORE_PAPER_SIDE_CHANNEL_H
#define PMSF_CORE_PAPER_SIDE_CHANNEL_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace pmsf {

template <typename SideMap, typename Cell>
inline void appendPaperSideCellSideMap(SideMap& side,
                                       const std::string& name,
                                       Cell&& value)
{
    side[name].push_back(std::forward<Cell>(value));
}

template <typename RoundSymbols, typename Cell>
inline void appendPaperSideCellSideChannel(RoundSymbols& round,
                                           const std::string& name,
                                           Cell&& value)
{
    appendPaperSideCellSideMap(round.side, name, std::forward<Cell>(value));
}

template <typename PaperRoundSymbols, typename SideMap>
inline void replacePMSFSideChannel(PaperRoundSymbols& delta,
                                    const SideMap& side)
{
    delta.side = side;
}

template <typename TargetRoundSymbols, typename SourceRoundSymbols>
inline void replacePMSFSideChannelFromRound(TargetRoundSymbols& target,
                                             const SourceRoundSymbols& source)
{
    replacePMSFSideChannel(target, source.side);
}

template <typename PaperRoundSymbols>
inline void erasePMSFSideChannel(PaperRoundSymbols& delta,
                                  const std::string& name)
{
    delta.side.erase(name);
}

template <typename RoundSymbols>
inline bool paperSideChannelHasCells(const RoundSymbols& round,
                                     const std::string& name)
{
    const auto it = round.side.find(name);
    return it != round.side.end() && !it->second.empty();
}

template <typename RoundSymbols>
inline bool paperSideChannelsEmpty(const RoundSymbols& round)
{
    return round.side.empty();
}

template <typename RoundSymbols>
inline void reservePMSFSideChannelCells(RoundSymbols& round,
                                         const std::string& name,
                                         std::size_t additional)
{
    auto& cells = round.side[name];
    cells.reserve(cells.size() + additional);
}

template <typename RoundSymbols>
inline std::size_t paperSideChannelCellCount(const RoundSymbols& round,
                                             const std::string& name)
{
    const auto it = round.side.find(name);
    return it == round.side.end() ? 0u : it->second.size();
}

template <typename RoundSymbols, typename Callback>
inline bool visitPMSFSideChannelCellAt(const RoundSymbols& round,
                                        const std::string& name,
                                        std::size_t index,
                                        Callback&& callback)
{
    const auto it = round.side.find(name);
    if (it == round.side.end() || index >= it->second.size()) return false;
    callback(it->second[index]);
    return true;
}

template <typename RoundSymbols, typename Callback>
inline void forEachPMSFSideChannelCell(const RoundSymbols& round,
                                        const std::string& name,
                                        Callback&& callback)
{
    const auto it = round.side.find(name);
    if (it == round.side.end()) return;
    for (const auto& cell : it->second) {
        callback(cell);
    }
}

template <typename RoundSymbols, typename Callback>
inline void forEachPMSFSideChannel(const RoundSymbols& round,
                                    Callback&& callback)
{
    for (const auto& entry : round.side) {
        callback(entry.first, entry.second);
    }
}

template <typename SideMap>
inline void appendPaperI32SideMap(SideMap& side,
                                  const std::string& name,
                                  std::int32_t value)
{
    appendPaperSideCellSideMap(side, name, value);
}

template <typename SideMap>
inline void appendPaperI32SideMap(SideMap& side,
                                  const std::string& prefix,
                                  const std::string& suffix,
                                  std::int32_t value)
{
    appendPaperI32SideMap(side, prefix + "_" + suffix, value);
}

template <typename PaperRoundSymbols>
inline void appendPaperI32SideChannel(PaperRoundSymbols& delta,
                                      const std::string& name,
                                      std::int32_t value)
{
    appendPaperI32SideMap(delta.side, name, value);
}

template <typename PaperRoundSymbols>
inline void appendPaperI32SideChannel(PaperRoundSymbols& delta,
                                      const std::string& prefix,
                                      const std::string& suffix,
                                      std::int32_t value)
{
    appendPaperI32SideMap(delta.side, prefix, suffix, value);
}

template <typename SideMap, typename Entries>
inline void appendPaperI32SideMapEntries(SideMap& side,
                                         const Entries& entries)
{
    for (const auto& entry : entries) {
        appendPaperI32SideMap(side, entry.first, entry.second);
    }
}

template <typename PaperRoundSymbols, typename Entries>
inline void appendPaperI32SideChannelEntries(PaperRoundSymbols& delta,
                                             const Entries& entries)
{
    appendPaperI32SideMapEntries(delta.side, entries);
}

template <typename SideMap, typename Values>
inline void appendPaperI32SideMapValues(SideMap& side,
                                        const std::string& name,
                                        const Values& values)
{
    auto& out = side[name];
    out.reserve(out.size() + values.size());
    for (const auto& value : values) {
        out.push_back(static_cast<std::int32_t>(value));
    }
}

template <typename PaperRoundSymbols, typename Values>
inline void appendPaperI32SideChannelValues(PaperRoundSymbols& delta,
                                            const std::string& name,
                                            const Values& values)
{
    appendPaperI32SideMapValues(delta.side, name, values);
}

template <typename SideMap, typename Values>
inline void replacePaperI32SideMap(SideMap& side,
                                   const std::string& name,
                                   const Values& values)
{
    auto& out = side[name];
    out.clear();
    out.reserve(values.size());
    for (const auto& value : values) {
        out.push_back(static_cast<std::int32_t>(value));
    }
}

template <typename SideMap, typename Values>
inline void replacePaperI32SideMap(SideMap& side,
                                   const std::string& prefix,
                                   const std::string& suffix,
                                   const Values& values)
{
    replacePaperI32SideMap(side, prefix + "_" + suffix, values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replacePaperI32SideChannel(PaperRoundSymbols& delta,
                                       const std::string& name,
                                       const Values& values)
{
    replacePaperI32SideMap(delta.side, name, values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replacePaperI32SideChannel(PaperRoundSymbols& delta,
                                       const std::string& prefix,
                                       const std::string& suffix,
                                       const Values& values)
{
    replacePaperI32SideMap(delta.side, prefix, suffix, values);
}

template <typename SideMap>
inline void appendPaperI8SideMap(SideMap& side,
                                 const std::string& name,
                                 std::int8_t value)
{
    appendPaperSideCellSideMap(side, name, value);
}

template <typename SideMap>
inline void appendPaperI8SideMap(SideMap& side,
                                 const std::string& prefix,
                                 const std::string& suffix,
                                 std::int8_t value)
{
    appendPaperI8SideMap(side, prefix + "_" + suffix, value);
}

template <typename PaperRoundSymbols>
inline void appendPaperI8SideChannel(PaperRoundSymbols& delta,
                                     const std::string& name,
                                     std::int8_t value)
{
    appendPaperI8SideMap(delta.side, name, value);
}

template <typename PaperRoundSymbols>
inline void appendPaperI8SideChannel(PaperRoundSymbols& delta,
                                     const std::string& prefix,
                                     const std::string& suffix,
                                     std::int8_t value)
{
    appendPaperI8SideMap(delta.side, prefix, suffix, value);
}

template <typename SideMap>
inline void appendPaperF32SideMap(SideMap& side,
                                  const std::string& name,
                                  float value)
{
    appendPaperSideCellSideMap(side, name, value);
}

template <typename SideMap>
inline void appendPaperF32SideMap(SideMap& side,
                                  const std::string& prefix,
                                  const std::string& suffix,
                                  float value)
{
    appendPaperF32SideMap(side, prefix + "_" + suffix, value);
}

template <typename PaperRoundSymbols>
inline void appendPaperF32SideChannel(PaperRoundSymbols& delta,
                                      const std::string& name,
                                      float value)
{
    appendPaperF32SideMap(delta.side, name, value);
}

template <typename PaperRoundSymbols>
inline void appendPaperF32SideChannel(PaperRoundSymbols& delta,
                                      const std::string& prefix,
                                      const std::string& suffix,
                                      float value)
{
    appendPaperF32SideMap(delta.side, prefix, suffix, value);
}

} // namespace pmsf

#endif // PMSF_CORE_PAPER_SIDE_CHANNEL_H
