import datetime

# 读取文件并处理波形（复用之前代码）
with open('data.raw', 'r') as f:
    hex_list = f.read().strip().split()

headers_order = ['70', '71', '72', '80']
headers = {k: [] for k in headers_order}
current_header = None

for hex_byte in hex_list:
    if hex_byte in headers:
        current_header = hex_byte
    else:
        if current_header is not None:
            headers[current_header].append(hex_byte)

waveforms = {}
for header in headers_order:
    data = headers[header]
    binary_str = ''.join([format(int(b,16), '08b')[::-1] for b in data])
    waveforms[header] = binary_str

# 验证所有波形长度一致
wave_lens = set(len(w) for w in waveforms.values())
if len(wave_lens) > 1:
    raise ValueError("波形长度不一致，无法合并")

# 生成合并VCD
def generate_merged_vcd(waveforms):
    timestamp = 0
    prev_values = {h: None for h in headers_order}
    time_records = []
    max_len = len(next(iter(waveforms.values())))
    
    # 遍历每个时间点
    for t in range(max_len):
        changes = []
        # 检查每个信号是否变化
        for header in headers_order:
            current_bit = waveforms[header][t]
            if prev_values[header] != current_bit:
                changes.append((header, current_bit))
                prev_values[header] = current_bit
        # 记录变化
        if changes:
            time_records.append((t, changes))
    
    # 生成VCD内容
    vcd = [
        f"$date\n    {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}\n$end",
        "$timescale 1ms $end",
        "$scope module logic $end",
    ]
    
    # 声明信号变量（每个header对应唯一标识符）
    var_symbols = {
        '70': 's0',
        '71': 's1',
        '72': 's2',
        '80': 's3'
    }
    for header, symbol in var_symbols.items():
        vcd.append(f"$var wire 1 {symbol} {header}_signal $end")
    
    vcd.extend([
        "$upscope $end",
        "$enddefinitions $end",
        "#0"  # 初始时间点
    ])
    
    # 初始化所有信号值
    for header in headers_order:
        initial_value = waveforms[header][0]
        vcd.append(f"{initial_value}{var_symbols[header]}")
    
    # 写入时间变化记录
    for t, changes in time_records:
        vcd.append(f"#{t+1}")  # 时间从1ms开始（0已初始化）
        for header, value in changes:
            vcd.append(f"{value}{var_symbols[header]}")
    
    return '\n'.join(vcd)

# 写入合并的VCD文件
with open('merged_signals.vcd', 'w') as f:
    f.write(generate_merged_vcd(waveforms))

print("Merged VCD file generated: merged_signals.vcd")
