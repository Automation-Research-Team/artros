#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from scipy import signal
from scipy import fftpack
from matplotlib import pyplot as plt

#バターワースフィルタ（ローパス）
def lowpass(x, samplerate, fp, fs, gpass, gstop):
    fn = samplerate / 2             #ナイキスト周波数
    wp = fp / fn                    #ナイキスト周波数で通過域端周波数を正規化
    ws = fs / fn                    #ナイキスト周波数で阻止域端周波数を正規化
    #オーダーとバターワースの正規化周波数を計算
    N, Wn = signal.buttord(wp, ws, gpass, gstop)
    b, a = signal.butter(N, Wn, "low")    #フィルタ伝達関数の分子と分母を計算
    y = signal.filtfilt(b, a, x)          #信号に対してフィルタをかける
    return y                              #フィルタ後の信号を返す

# オーバーラップ処理
def ov(data, samplerate, Fs, overlap):
    Ts = len(data) / samplerate           # 全データ長
    Fc = Fs / samplerate                  # フレーム周期
    x_ol = Fs * (1 - (overlap / 100))     # オーバーラップ時のフレームずらし幅
    # 抽出するフレーム数（平均化に使うデータ個数）
    N_ave = int((Ts - (Fc * (overlap / 100))) / (Fc * (1 - (overlap / 100))))

    array = []                            # 抽出したデータを入れる空配列の定義

    # forループでデータを抽出
    for i in range(N_ave):
        ps = int(x_ol * i)                # 切り出し位置をループ毎に更新
        # 切り出し位置psからフレームサイズ分抽出して配列に追加
        array.append(data[ps:ps + Fs:1])
    # オーバーラップ抽出されたデータ配列とデータ個数を戻り値にする
    return array, N_ave

# 窓関数処理（ハニング窓）
def hanning(data_array, Fs, N_ave):
    han = signal.hann(Fs)        # ハニング窓作成
    acf = 1 / (sum(han) / Fs)    # 振幅補正係数(Amplitude Correction Factor)

    # オーバーラップされた複数時間波形全てに窓関数をかける
    for i in range(N_ave):
        data_array[i] = data_array[i] * han  # 窓関数をかける

    return data_array, acf

# FFT処理
def fft_ave(data_array, samplerate, Fs, N_ave, acf):
    fft_array = []
    for i in range(N_ave):
        # FFTをして配列に追加、窓関数補正値をかけ、(Fs/2)の正規化を実施
        fft_array.append(acf * np.abs(fftpack.fft(data_array[i]) / (Fs / 2)))

    fft_axis = np.linspace(0, samplerate, Fs)  # 周波数軸を作成
    fft_array = np.array(fft_array)            # 型をndarrayに変換
    fft_mean = np.mean(fft_array, axis=0)      # 全てのFFT波形の平均を計算

    return fft_array, fft_mean, fft_axis

#リニア値からdBへ変換
def db(x, dBref):
    y = 20 * np.log10(x / dBref)     #変換式
    return y                         #dB値を返す

# ここからサンプル波形生成とフィルタ処理をする------------------------------
samplerate = 25600
x = np.arange(0, 12800) / samplerate       # 波形生成のための時間軸の作成
data = np.random.normal(loc=0, scale=1, size=len(x))  # ガウシアンノイズを生成

fp = 3000                                               # 通過域端周波数[Hz]
fs = 6000                                               # 阻止域端周波数[Hz]
gpass = 3                                               # 通過域端最大損失[dB]
gstop = 40                                              # 阻止域端最小損失[dB]

# ローパスをする関数を実行
data_filt = lowpass(data, samplerate, fp, fs, gpass, gstop)
# --------------------------------------------------------------------------

# ここから平均化フーリエ変換とデシベル変換を行う----------------------------
Fs = 4096                                               # フレームサイズ
overlap = 90                                            # オーバーラップ率

# 作成した関数を実行：オーバーラップ抽出された時間波形配列
t_array_org,  N_ave_org  = ov(data,      samplerate, Fs, overlap)
t_array_filt, N_ave_filt = ov(data_filt, samplerate, Fs, overlap)

# 作成した関数を実行：ハニング窓関数をかける
t_array_org,  acf_org  = hanning(t_array_org,  Fs, N_ave_org)
t_array_filt, acf_filt = hanning(t_array_filt, Fs, N_ave_filt)

# 作成した関数を実行：FFTをかける
fft_array_org,  fft_mean_org,  fft_axis_org  = fft_ave(t_array_org,
                                                       samplerate, Fs,
                                                       N_ave_org, acf_org)
fft_array_filt, fft_mean_filt, fft_axis_filt = fft_ave(t_array_filt,
                                                       samplerate, Fs,
                                                       N_ave_filt, acf_filt)

# dB変換する
fft_mean_org  = db(fft_mean_org,  2e-5)
fft_mean_filt = db(fft_mean_filt, 2e-5)
# --------------------------------------------------------------------------

# ここからグラフ描画--------------------------------------------------------
# フォントの種類とサイズを設定する。
plt.rcParams['font.size'] = 14
plt.rcParams['font.family'] = 'Times New Roman'

# 目盛を内側にする。
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'

# グラフの上下左右に目盛線を付ける。
fig = plt.figure()
ax1 = fig.add_subplot(211)
ax1.yaxis.set_ticks_position('both')
ax1.xaxis.set_ticks_position('both')
ax2 = fig.add_subplot(212)
ax2.yaxis.set_ticks_position('both')
ax2.xaxis.set_ticks_position('both')

# 軸のラベルを設定する。
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('SP[Pa]')
ax2.set_xlabel('Frequency [Hz]')
ax2.set_ylabel('SPL[dB]')

# データプロットの準備とともに、ラベルと線の太さ、凡例の設置を行う。
ax1.plot(x, data, label='original', lw=1)
ax1.plot(x, data_filt, label='filtered', lw=1)
ax2.plot(fft_axis_org, fft_mean_org, label='original', lw=1)
ax2.plot(fft_axis_filt, fft_mean_filt, label='filtered', lw=1)
plt.legend()

# 軸のリミットを設定する。
ax2.set_xlim(0, max(fft_axis_org)/2)
ax2.set_ylim(-20, 100)

# レイアウト設定
fig.tight_layout()

# グラフを表示する。
plt.show()
plt.close()
# -----------------------------------------------------------------------------------
