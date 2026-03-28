import os
import time
from blf_to_mf4 import blf_to_mf4

DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data')
DATA_DIR = os.path.abspath(DATA_DIR)

# DBC mapping used by project (adjust if needed)
DBC_FILES = {0: os.path.join(os.path.dirname(__file__), '..', 'dbc', 'SAC', 'target_CHCANFD.dbc'),
             1: os.path.join(os.path.dirname(__file__), '..', 'dbc', 'SAC', 'target_SFCANFD.dbc')}

LOG_FILE = os.path.join(DATA_DIR, 'batch_conversion.log')

if __name__ == '__main__':
    os.makedirs(DATA_DIR, exist_ok=True)
    blf_files = [f for f in os.listdir(DATA_DIR) if f.lower().endswith('.blf')]
    blf_files.sort()

    with open(LOG_FILE, 'a', encoding='utf-8') as log:
        log.write(f"\nBatch run started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        for blf in blf_files:
            blf_path = os.path.join(DATA_DIR, blf)
            mf4_name = os.path.basename(blf).rsplit('.', 1)[0] + '.mf4'
            mf4_path = os.path.join(DATA_DIR, mf4_name)

            # skip if mf4 exists and is newer than blf
            if os.path.exists(mf4_path) and os.path.getmtime(mf4_path) >= os.path.getmtime(blf_path):
                log.write(f"SKIP (exists): {blf} -> {mf4_name}\n")
                print(f"SKIP (exists): {blf} -> {mf4_name}")
                continue

            print(f"Converting: {blf_path} -> {mf4_path}")
            log.write(f"Converting: {blf_path} -> {mf4_path}\n")
            start = time.time()
            try:
                rc = blf_to_mf4(blf_file=blf_path, db_file=DBC_FILES, mf4_file=mf4_path)
                duration = time.time() - start
                if rc == 0:
                    log.write(f"OK: {blf} -> {mf4_name} ({duration:.1f}s)\n")
                    print(f"OK: {blf} -> {mf4_name} ({duration:.1f}s)")
                else:
                    log.write(f"FAIL (rc {rc}): {blf} -> {mf4_name} ({duration:.1f}s)\n")
                    print(f"FAIL (rc {rc}): {blf} -> {mf4_name} ({duration:.1f}s)")
            except Exception as e:
                duration = time.time() - start
                log.write(f"EXCEPTION: {blf} -> {mf4_name} ({duration:.1f}s): {e}\n")
                print(f"EXCEPTION: {blf} -> {mf4_name} ({duration:.1f}s): {e}")
        log.write(f"Batch run finished: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    print('Batch conversion complete. See', LOG_FILE)
