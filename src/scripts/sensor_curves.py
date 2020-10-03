velodyne_vert = [1.70/(i*float(ds)*math.tan(math.radians(2))) + 1 for i in range(40,n)]
velodyne_horiz = [0.5 / (i * float(ds) * math.tan(math.radians(0.2))) + 1 for i in range(40, n)]
velodyne = np.array(velodyne_vert) + np.array(velodyne_horiz)
hdl64_vert = [1.70 / (i * float(ds) * math.tan(math.radians(0.4))) + 1 for i in range(40, n)]
hdl64_horiz = [0.5 / (i * float(ds) * math.tan(math.radians(0.17))) + 1 for i in range(40, n)]
hdl64 = np.array(hdl64_vert) + np.array(hdl64_horiz)
plt.plot(list_of_ds[40:], velodyne, label='Puck')
plt.plot(list_of_ds[40:], hdl64, label='HDL-64E')
plt.ylabel('points/object')
plt.xlabel('d [m]')
plt.legend(loc="upper right")
plt.savefig('Lidar-Performance.png')
plt.show()
recall, precision = get_recall_precision(str("faster_rcnn96"), n, ds)
f_recall = interp1d(range(n), recall)
f_precision = interp1d(range(n), precision)
cam_basler = CameraSpecification(f=Decimal(str(4.0)), height=Decimal(str(4.55)), width=Decimal(str(6.17)),
                             n_pixel_height=1920, n_pixel_width=1200)
cam_flea = CameraSpecification(f=Decimal(str(4.0)), height=Decimal(str(6.2)), width=Decimal(str(7.6)),
                                 n_pixel_height=1384, n_pixel_width=1032)

rfov_basler = cam_basler.get_vertical_fov(list_of_ds)
resolution_basler = cam_basler.get_vertical_resolution(rfov_basler)
recall_basler = [f_recall(float(resolution_basler[i])) for i in range(len(resolution_basler))]
precision_basler = [f_precision(float(resolution_basler[i])) for i in range(len(resolution_basler))]
fn_basler = 1 - np.array(recall_basler)
fp_basler = 1 - np.array(precision_basler)

rfov_flea = cam_flea.get_vertical_fov(list_of_ds)
resolution_flea = cam_flea.get_vertical_resolution(rfov_flea)
recall_flea = [f_recall(float(resolution_flea[i])) for i in range(len(resolution_flea))]
precision_flea = [f_precision(float(resolution_flea[i])) for i in range(len(resolution_flea))]
fn_flea = 1 - np.array(recall_flea)
fp_flea = 1 - np.array(precision_flea)

plt.plot(fp_basler, fn_basler, label='Basler')
plt.plot(fp_flea, fn_flea, label='FLIR')
plt.ylabel('FN')
plt.xlabel('FP')
plt.legend(loc="upper right")
plt.savefig('FN-FP.png')
plt.show()

fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2)
fig.suptitle('Recall, Precision, FPR, FNR dependent on distance/object height')
ax1.plot(range(len(recall)), recall)
ax1.set(xlabel='Object height in pixel', ylabel='Recall')
ax2.plot(range(len(precision)), precision)
ax2.set(xlabel='Object height in pixel', ylabel='Precision')
ax3.plot(list_of_ds, recall_basler, color="red", label="Basler")
ax3.plot(list_of_ds, recall_flea, color="blue", label="FLIR")
ax3.legend(loc="upper right")
ax3.set(xlabel='Distance in [m]', ylabel='Recall')
ax4.plot(list_of_ds, precision_basler, color="red", label="Basler")
ax4.plot(list_of_ds, precision_flea, color="blue", label="FLIR")
ax4.legend(loc="upper right")
ax4.set(xlabel='Distance in [m]', ylabel='Precision')
ax5.plot(list_of_ds, fn_basler, color="red", label="Basler")
ax5.plot(list_of_ds, fn_flea, color="blue", label="FLIR")
ax5.legend(loc="lower right")
ax5.set(xlabel='Distance in [m]', ylabel='FNR')
ax6.plot(list_of_ds, fp_basler, color="red", label="Basler")
ax6.plot(list_of_ds, fp_flea, color="blue", label="FLIR")
ax6.legend(loc="lower right")
ax6.set(xlabel='Distance in [m]', ylabel='FPR')
plt.tight_layout()
plt.savefig('Camera-Performance.png')
plt.show()
