import cv2
import numpy as np
from ultralytics import YOLO
import time
import argparse
import os
from tqdm import tqdm

def process_video(video_path, model_path, conf_thresh=0.4, save_output=True, output_path="output.mp4"):
    # Load model YOLOv8
    print(f"Loading model {model_path}...")
    model = YOLO(model_path)
    
    # Buka file video
    if not os.path.exists(video_path):
        print(f"Error: File video tidak ditemukan: {video_path}")
        return
    
    print(f"Membuka video: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Tidak dapat membuka video: {video_path}")
        return
    
    # Dapatkan properti video
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Resolusi video: {frame_width}x{frame_height}, FPS: {fps}, Total frames: {total_frames}")
    
    # Setup video writer
    out = None
    if save_output:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))
        print(f"Output akan disimpan ke: {output_path}")
    
    # Sesuaikan checkpoint area berdasarkan resolusi video
    checkpoints = {
        "Area 1": [int(frame_width*0.7), 0, frame_width, int(frame_height*0.3)],
        "Area 2": [int(frame_width*0.3), int(frame_height*0.3), int(frame_width*0.7), int(frame_height*0.7)],
        "Area 3": [0, int(frame_height*0.7), int(frame_width*0.3), frame_height]
    }
    
    # Inisialisasi warna untuk setiap area (B, G, R)
    colors = {
        "Area 1": (0, 255, 0),    # Hijau
        "Area 2": (0, 255, 255),  # Kuning
        "Area 3": (255, 0, 0)     # Biru
    }
    
    # Struktur data untuk mencatat objek dan waktu
    object_in_checkpoint = {area: set() for area in checkpoints}
    object_entry_time = {area: {} for area in checkpoints}
    active_timers = {area: {} for area in checkpoints}
    
    # Log file untuk mencatat hasil
    log_file = open("detection_log.txt", "w")
    log_file.write(f"Pengujian Model: {model_path} pada Video: {video_path}\n")
    log_file.write(f"Timestamp\tArea\tObjek\tAksi\tDurasi\n")
    
    # Function untuk mengecek apakah bbox objek berada di dalam checkpoint
    def is_in_area(bbox, area):
        x1, y1, x2, y2 = bbox
        ax1, ay1, ax2, ay2 = area
        
        # Cek apakah tengah bounding box berada di dalam area
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        return (ax1 <= center_x <= ax2) and (ay1 <= center_y <= ay2)
    
    # Fungsi untuk menggambar checkpoint pada frame
    def draw_checkpoints(frame):
        for name, area in checkpoints.items():
            x1, y1, x2, y2 = area
            cv2.rectangle(frame, (x1, y1), (x2, y2), colors[name], 2)
            cv2.putText(frame, name, (x1, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[name], 2)
        
        return frame
    
    # Inisialisasi progress bar
    pbar = tqdm(total=total_frames, desc="Processing video")
    
    # Variabel untuk waktu video
    frame_number = 0
    
    # Loop utama pemrosesan video
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("\nPemrosesan video selesai.")
            break
        
        # Update progress bar
        pbar.update(1)
        frame_number += 1
        
        # Konversi nomor frame ke timestamp video (hh:mm:ss)
        video_time = frame_number / fps
        hours = int(video_time // 3600)
        minutes = int((video_time % 3600) // 60)
        seconds = int(video_time % 60)
        timestamp = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
        
        # Tambahkan timestamp ke frame
        cv2.putText(frame, f"Time: {timestamp}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Gambar checkpoint areas
        frame = draw_checkpoints(frame)
        
        # Jalankan deteksi YOLOv8
        results = model(frame, conf=conf_thresh)
        
        # Simpan deteksi sebelumnya untuk dibandingkan nanti
        previous_detections = {area: object_in_checkpoint[area].copy() for area in checkpoints}
        
        # Reset deteksi untuk frame ini
        current_detections = {area: set() for area in checkpoints}
        
        # Proses hasil deteksi
        for r in results:
            boxes = r.boxes
            
            for box in boxes:
                # Ekstrak bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                # Dapatkan nama kelas
                class_name = model.names[cls]
                
                # Gambar bounding box untuk semua objek yang terdeteksi
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                
                # Buat ID unik untuk objek
                object_id = f"{class_name}_{x1}_{y1}"
                
                # Cek apakah objek berada di dalam checkpoint
                for area_name, area_coords in checkpoints.items():
                    if is_in_area([x1, y1, x2, y2], area_coords):
                        current_detections[area_name].add(object_id)
                        
                        # Objek baru masuk ke checkpoint - mulai timer
                        if object_id not in object_in_checkpoint[area_name]:
                            object_entry_time[area_name][object_id] = video_time
                            active_timers[area_name][object_id] = True
                            
                            # Log entry event
                            log_entry = f"{timestamp}\t{area_name}\t{class_name}\tMasuk\t0.00\n"
                            log_file.write(log_entry)
                            
                        # Highlight objek yang ada di checkpoint
                        cv2.rectangle(frame, (x1, y1), (x2, y2), colors[area_name], 3)
                        
                        # Tampilkan timer untuk objek yang berada di dalam area
                        if object_id in object_entry_time[area_name]:
                            time_in_area = video_time - object_entry_time[area_name][object_id]
                            timer_text = f"{time_in_area:.1f}s"
                            cv2.putText(frame, timer_text, (x1, y2+20), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[area_name], 2)
        
        # Cek objek yang keluar dari area checkpoint
        for area_name in checkpoints:
            # Objek yang ada di frame sebelumnya tetapi tidak ada di frame saat ini
            exited_objects = previous_detections[area_name] - current_detections[area_name]
            
            for object_id in exited_objects:
                if object_id in object_entry_time[area_name] and active_timers[area_name].get(object_id, False):
                    entry_time = object_entry_time[area_name][object_id]
                    duration = video_time - entry_time
                    
                    # Ambil nama kelas dari object_id (format: class_name_x_y)
                    class_name = object_id.split('_')[0]
                    
                    # Log exit event
                    log_entry = f"{timestamp}\t{area_name}\t{class_name}\tKeluar\t{duration:.2f}\n"
                    log_file.write(log_entry)
                    
                    # Nonaktifkan timer
                    active_timers[area_name][object_id] = False
        
        # Update tabel deteksi
        object_in_checkpoint = current_detections
        
        # Tampilkan jumlah objek di setiap area
        y_offset = 60
        for area_name, detections in current_detections.items():
            count = len(detections)
            cv2.putText(frame, f"{area_name}: {count} objek", 
                        (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[area_name], 2)
            y_offset += 30
        
        # Tampilkan progress
        cv2.putText(frame, f"Progress: {frame_number}/{total_frames} ({frame_number/total_frames*100:.1f}%)", 
                    (10, frame_height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Simpan frame ke video output
        if save_output and out is not None:
            out.write(frame)
        
        # Tampilkan frame (opsional, bisa dikomentari untuk pemrosesan lebih cepat)
        cv2.imshow("Pengujian Model pada Video", frame)
        
        # Kontrol keyboard
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):  # Pause/play
            while True:
                key2 = cv2.waitKey(0) & 0xFF
                if key2 == ord('p'):
                    break
                elif key2 == ord('q'):
                    cap.release()
                    if out is not None:
                        out.release()
                    cv2.destroyAllWindows()
                    log_file.close()
                    return
    
    # Tutup progress bar
    pbar.close()
    
    # Pembersihan
    cap.release()
    if out is not None:
        out.release()
    cv2.destroyAllWindows()
    
    # Tutup log file
    log_file.close()
    print(f"Log deteksi tersimpan di: detection_log.txt")

if __name__ == "__main__":
    # Parse argumen command line
    parser = argparse.ArgumentParser(description="Pengujian Model YOLOv8 pada Video")
    parser.add_argument("--video", type=str, required=True, help="Path ke file video")
    parser.add_argument("--model", type=str, default="best.pt", help="Path ke model YOLOv8 (.pt file)")
    parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold")
    parser.add_argument("--no-save", action="store_true", help="Jangan simpan output video")
    parser.add_argument("--output", type=str, default="hasil_deteksi.mp4", help="Nama file output")
    
    args = parser.parse_args()
    
    # Proses video
    process_video(
        video_path=args.video,
        model_path=args.model,
        conf_thresh=args.conf,
        save_output=not args.no_save,
        output_path=args.output
    )
