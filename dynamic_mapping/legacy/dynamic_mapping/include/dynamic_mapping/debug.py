import matplotlib.figure as mplfig


class ImgVis:
    def __init__(self):
        self.fig = mplfig.Figure(frameon=False)
        dpi = self.fig.get_dpi()
        self.fig.set_size_inches(1376.01 / dpi, 1152.01 / dpi)
        self.fig.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)

    def save_projection(self, visible_points, savepath, bg_img):
        u, v, z = visible_points
        ax = self.fig.add_axes([0.0, 0.0, 1.0, 1.0])
        ax.axis("off")
        ax.set_frame_on(False)
        ax.imshow(bg_img)
        ax.scatter([u], [v], c=[z], cmap="rainbow_r", s=3, vmin=0, vmax=20)
        ax.margins(0, 0)
        self.fig.savefig(savepath)
        self.fig.clf()
