import can
import cantools
import pandas as pd
from datetime import datetime
import asammdf as mf4
import numpy as np
import os
def blf_to_mf4(blf_file:str=None, db_file:dict=None,mf4_file:str=None,no_attribute_parse:list=None):

    db_set = dict()
    if db_file is None:
        print("⚠ 警告：未提供DBC文件，无法进行BLF到MF4的转换")
        return -1
    for db_key in db_file:
        db_set[db_key] = cantools.database.load_file(db_file[db_key])
    log = can.BLFReader(blf_file)
    # 按channel和message id组织数据
    channel_data = dict()  # {channel: {msg_id: [dict(signal1=val1, ..., timestamp=ts), ...]}}
    db_messages = dict()
    for message in log:
        if isinstance(message.timestamp, datetime):
            timestamp = float(message.timestamp.timestamp())
        else:
            timestamp = float(message.timestamp)
        db = db_set.get(message.channel, None)
        if db is None:
            print(f"⚠ 警告：未找到通道 {message.channel} 对应的DBC文件，跳过该消息")
            continue
        try:
            db_message = db.get_message_by_frame_id(message.arbitration_id)
            if message is not None:
                decode_choices = True
                if no_attribute_parse is not None and db_message.name in no_attribute_parse:
                    decode_choices = False
                decoded_message = db.decode_message(message.arbitration_id, message.data,decode_choices=decode_choices)
            else:
                print(f"⚠ 警告：消息 ID 0x{message.arbitration_id:X} 解析错误，跳过该消息")
                continue
        except Exception as e:
            #print(f"⚠ 解码失败: {e}")
            continue
        # 按channel和msg_id分组
        # get message name
        
        message_name = db_message.name
        db_messages[message_name] = db_message
        if message_name not in channel_data:
            channel_data[message_name] = dict()
            keys = decoded_message.keys()
            for key in keys:
                channel_data[message_name][key] = []
            channel_data[message_name]['t'] = []
        for signal, value in decoded_message.items():
            if value is None:
                print(f"⚠ 警告：信号 {signal} 在消息 {message_name} 中的值为None，使用0代替")
                value = 0
            channel_data[message_name][signal].append(value)
        channel_data[message_name]['t'].append(timestamp)
    # 写入MF4
    with mf4.MDF(version="4.10") as writer:
        for message_name, msg_dict in channel_data.items():
            timestamp = msg_dict.pop('t')
            timestamp = np.asarray(timestamp, dtype=np.float64)
            #print(f'time:{timestamp}')
            message = db_messages[message_name]
            signals = []
            for signal_name, values in msg_dict.items():
                print('--- processing signal:{} ---'.format(signal_name))
                print('value:{},type:{}'.format(values[0], type(values[0])))
                db_signal = message.get_signal_by_name(signal_name)
                is_string_type = False

                def safe_int_convert(x):
                    if x is None:
                        return 0
                    if isinstance(x, bool):
                        return int(x)
                    if isinstance(x, (int, np.integer)):
                        val = int(x)
                        # clip to int64 range
                        max_i64 = np.iinfo(np.int64).max
                        min_i64 = np.iinfo(np.int64).min
                        if val > max_i64:
                            return int(max_i64)
                        if val < min_i64:
                            return int(min_i64)
                        return val
                    if isinstance(x, (float, np.floating)):
                        val = int(x)
                        max_i64 = np.iinfo(np.int64).max
                        min_i64 = np.iinfo(np.int64).min
                        if val > max_i64:
                            return int(max_i64)
                        if val < min_i64:
                            return int(min_i64)
                        return val
                    val_attr = getattr(x, 'value', None)
                    if isinstance(val_attr, (int, np.integer)):
                        val = int(val_attr)
                        max_i64 = np.iinfo(np.int64).max
                        min_i64 = np.iinfo(np.int64).min
                        if val > max_i64:
                            return int(max_i64)
                        if val < min_i64:
                            return int(min_i64)
                        return val
                    try:
                        val = int(x)
                        max_i64 = np.iinfo(np.int64).max
                        min_i64 = np.iinfo(np.int64).min
                        if val > max_i64:
                            return int(max_i64)
                        if val < min_i64:
                            return int(min_i64)
                        return val
                    except Exception:
                        try:
                            val = int(str(x))
                            max_i64 = np.iinfo(np.int64).max
                            min_i64 = np.iinfo(np.int64).min
                            if val > max_i64:
                                return int(max_i64)
                            if val < min_i64:
                                return int(min_i64)
                            return val
                        except Exception:
                            return 0

                # 全为字符串 -> 字符串序列
                if isinstance(values[0], str) or all(isinstance(v, str) for v in values):
                    samples = values
                    is_string_type = True

                # 含有 name 属性（例如 NamedSignalValue） -> 使用 name
                elif any(hasattr(v, 'name') for v in values):
                    samples = []
                    for v in values:
                        name = getattr(v, 'name', None)
                        if name is None:
                            val = getattr(v, 'value', None)
                            samples.append(str(val) if val is not None else "unknown")
                        else:
                            if not isinstance(name, str):
                                name = str(name)
                            samples.append(name)
                    is_string_type = True

                # 全为浮点数
                elif all(isinstance(v, float) or isinstance(v, np.floating) for v in values):
                    samples = np.asarray(values, dtype=np.float64)

                # 全为整数/布尔 -> 转为整型数组（逐项安全转换）
                elif all(isinstance(v, (int, np.integer, bool)) for v in values):
                    samples = np.asarray([safe_int_convert(v) for v in values], dtype=np.int64)

                # 混合或未知类型：优先尝试数值转换，失败则降级为字符串
                else:
                    try:
                        samples = np.asarray([safe_int_convert(v) for v in values], dtype=np.int64)
                    except Exception:
                        samples = [str(v) for v in values]
                        is_string_type = True

                print(f'signal name:{signal_name}, unit:{db_signal.unit if db_signal and db_signal.unit else ""}')
                if is_string_type:
                    signal = mf4.Signal(
                        name=signal_name,
                        timestamps=timestamp,
                        samples=samples,
                        unit=db_signal.unit if db_signal and db_signal.unit else '',
                        comment=db_signal.comment if db_signal and db_signal.comment else '',
                        encoding='utf-8'   
                    )
                else:    
                    signal = mf4.Signal(
                        name=signal_name,
                        timestamps=timestamp,
                        samples=samples,
                        unit=db_signal.unit if db_signal and db_signal.unit else '',
                        comment=db_signal.comment if db_signal and db_signal.comment else ''
                    )
                signals.append(signal)
            writer.append(signals,common_timebase=True)
            cg = writer.groups[-1]
            cg.channel_group.comment = message_name
        writer.save(mf4_file, overwrite=True)
    return 0

if __name__ == "__main__":
    
    dbc_files = {0:"dbc/SAC/target_CHCANFD.dbc",
                  1:"dbc/SAC/target_SFCANFD.dbc",
                  2:"dbc/RTK/CAN_Protocol 2.0_20210507-motorola.dbc"}
    save_path = "data/vehicle"
    # blf_file_dir = "data/001.blf"
    # mf4_file_name = os.path.basename(blf_file).replace(".blf",".mf4")
    # mf4_file = os.path.join(save_path,mf4_file_name)
    # print(f"正在转换文件: {blf_file} 到 {mf4_file}")
    # no_attribute_parse_list = ["InsStatus"]
    # blf_to_mf4(blf_file=blf_file,db_file=dbc_files,mf4_file=mf4_file,no_attribute_parse=no_attribute_parse_list )
    blf_file_dir="data"
    mf4_file_dir = "data/vehicle"
    for file in os.listdir(blf_file_dir):
        if file.endswith(".blf"):
            blf_file = os.path.join(blf_file_dir,file)
            mf4_file = os.path.join(mf4_file_dir,file.replace(".blf",".mf4"))
            print(f"正在转换文件: {blf_file} 到 {mf4_file}")
            no_attribute_parse_list = ["InsStatus"]
            blf_to_mf4(blf_file=blf_file,db_file=dbc_files,mf4_file=mf4_file,no_attribute_parse=no_attribute_parse_list )
    print("转换完成")
                
              
        

