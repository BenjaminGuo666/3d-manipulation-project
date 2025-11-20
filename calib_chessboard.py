#!/usr/bin/env python3
import cv2, glob, os, yaml, numpy as np
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="folder of chessboard images")
    ap.add_argument("--pattern_cols", type=int, default=9, help="inner corners on columns")
    ap.add_argument("--pattern_rows", type=int, default=6, help="inner corners on rows")
    ap.add_argument("--square", type=float, required=True, help="square size in meters, e.g., 0.024")
    ap.add_argument("--out", default="camera.yaml", help="output yaml")
    ap.add_argument("--viz", action="store_true", help="show detections")
    args = ap.parse_args()

    pattern_size = (args.pattern_cols, args.pattern_rows)
    # 生成世界坐标（Z=0 平面）
    objp = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)
    objp *= args.square

    objpoints, imgpoints = [], []
    images = sorted(glob.glob(os.path.join(args.images, "*.*")))
    h=w=None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    ok_cnt = 0
    for fn in images:
        print("processing", fn, flush=True)
        img = cv2.imread(fn)
        if img is None:
            continue

        # —— 自动把超大图缩到 <=2000px 长边做检测（再映射回原图）——
        gray_full = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray_full.shape[:2]
        max_side = max(h, w)
        scale = 2000.0 / max_side if max_side > 2000 else 1.0
        gray = cv2.resize(gray_full, (int(w * scale), int(h * scale))) if scale < 1.0 else gray_full

        # —— 更鲁棒的检测器：SB（OpenCV 4.5+）——
        flags_sb = cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
        ret, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags_sb)

        if not ret:
            # 退回经典检测器再试一次
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            ret2, corners2 = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
            ret, corners = ret2, corners2

        if ret:
            # 映射回原图尺寸并亚像素优化
            if scale < 1.0:
                corners = corners / scale
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            corners = cv2.cornerSubPix(gray_full, corners, (11, 11), (-1, -1), crit)

            objpoints.append(objp.copy())
            imgpoints.append(corners)
            ok_cnt += 1

            if args.viz:
                vis = img.copy()
                cv2.drawChessboardCorners(vis, pattern_size, corners, True)
                cv2.imshow("corners", vis);
                cv2.waitKey(1)
        else:
            print(f"[WARN] corners not found: {os.path.basename(fn)}")

    if ok_cnt < 8:
        raise RuntimeError("Not enough valid chessboard images (need >= 8).")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (gray.shape[1], gray.shape[0]), None, None,
        flags=cv2.CALIB_RATIONAL_MODEL
    )
    # 评估重投影误差
    errs = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        e = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)/len(proj)
        errs.append(float(e))
    mean_err = float(np.mean(errs))
    print(f"[OK] RMS from calibrateCamera: {ret:.4f}, mean reproj err: {mean_err:.4f} px")
    h, w = gray.shape[:2]
    # 保存 YAML
    data = {
        "image_width": int(w),
        "image_height": int(h),
        "camera_matrix": {"data": K.flatten().tolist()},
        "distortion_coefficients": {"data": dist.flatten().tolist()},  # k1 k2 p1 p2 k3 k4 k5 k6
        "dist_model": "opencv-rational",
        "square_size_m": args.square,
        "pattern_cols": args.pattern_cols,
        "pattern_rows": args.pattern_rows
    }
    with open(args.out, "w") as f:
        yaml.safe_dump(data, f)
    print(f"[SAVE] {args.out}")

    # 可选：去畸变示例图
    if images:
        sample = cv2.imread(images[0])
        newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w,h), 1)
        und = cv2.undistort(sample, K, dist, None, newK)
        cv2.imwrite(os.path.join(os.path.dirname(args.out), "undistort_sample.jpg"), und)
        print("[SAVE] undistort_sample.jpg")

    if args.viz:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
