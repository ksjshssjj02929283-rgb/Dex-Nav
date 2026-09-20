# 地图状态原子下发消息（latched: transient_local, depth 1, reliable）
# folder + name + epoch + stamp 一条消息原子下发，消除多 topic 时序竞争。
# epoch：单调递增版本号，任何一次地图加载（含同名重载 / 回退）都 +1；
#        消费方比对 epoch（而非 name）判断是否需要重载，从而不漏检同名内容变更。
#        0 表示尚未加载任何地图。不持久化，节点重启从 0 重新计数。
std_msgs/Header header
string map_folder
string map_name
uint64 epoch
