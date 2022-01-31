# Nursing
SNCT 2A27 
HIGASHI KAITO 
research repository

# 準備
## OpenPose
### 画像を入力にする時
~/OpenPoseで
> ./build/examples/openpose/openpose.bin --image_dir examples/run_media  --write_images ./write_images_folder/  --write_json ./output_json_folder/
> 
を打ち込む。

* --image_dir examples/run_media(入力画像のディレクトリ)
* --write_images ./write_images_folder/（推定結果の出力先）
* --write_json ./output_json_folder/（jsonファイル（骨格情報）の出力先）
### 動画を入力にする時
~/OpenPoseで
> ./build/examples/openpose/openpose.bin --video examples/media/IMG_1270.MOV --write_video ./write_video_folder/write_IMG1270.avi  --write_json ./output_IMG1270_json_folder/
> 
を打ち込む。
* --video examples/media/IMG_1270.MOV（入力動画）
* --write_video ./write_video_folder/write_IMG1270.avi（推定結果の出力先）
* --write_json ./output_IMG1270_json_folder/（jsonファイル（骨格情報）の出力先）
## 3d-pose-baseline
~/3d-pose-baselineで
> python3 src/my_trans_2.py --camera_frame --residual --batch_norm --dropout 0.5 --max_norm --evaluateActionWise --use_sh --epochs 200 --load 4874200 --pose_estimation_json ./output_json_folder/ --write_gif 
>
を打ち込む。
* --pose_estimation_json ./output_json_folder/（OpenPoseで出力したjsonファイルがあるディレクトリ）
  * ディレクトリ内全てのjsonファイルが読み込まれるので、推定対象のjsonファイルのみを置くようにすること
コマンドを実行すると、~/3d-pose-baseline/keypoint_output_folder内に、txtファイルとして三次元骨格のキーポイント（関節）位置が出力される。
### 出力ファイル名変更

### 


## RootNet

# 実行方法
~/BulletTest/Nursing内で、
> make 
> 
> ./main
>
を打ち込む。すると、作成する人体モデル数と読み込むtxtファイルが聞かれるので入力する。

**人体モデルは、現状1体か2体しか生成できないので注意**
**~/BulletTest/Nursing/keypoint_folder内にtxtファイルを置いておく**
> Please Input Humanoid Num -> 1 (or 2) 
> 
> Please Input FileName -> keypoint_filename
>
すると、txtファイル内の骨格情報を基に人体モデルで姿勢が再現される。また、緑，黃，赤，黒で力の大きさに応じた色が表示される。

# 補足

* 動画をフレーム毎に画像分割する方法
  
  長時間の動画だとフレーム毎にOpenPoseの出力を見たい場合が在る。動画をフレーム毎に分割するには
  > mkdir frames
  >
  > cd frames
  >
  > mv write_IMG_1238.avi（分割したい動画名） ./frames
  >
  > mplayer -vo jpeg write_IMG_1238.avi（jpegで全フレームを画像出力）


# 改善点
3d-pose-baselineのtxtファイル名を入力ファイル名から決定できるようにする。