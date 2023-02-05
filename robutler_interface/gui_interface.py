import tkinter
import tkinter.messagebox
import customtkinter
from PIL import Image
from geeteventbus.subscriber import subscriber
from geeteventbus.eventbus import eventbus
from geeteventbus.event import event

customtkinter.set_appearance_mode(
    "System"
)  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme(
    "blue"
)  # Themes: "blue" (standard), "green", "dark-blue"


class App(customtkinter.CTk):
    def __init__(self, bus):

        self.bus = bus
        super().__init__()

        # configure window
        self.title("Robutler App")
        self.geometry(f"{1100}x{580}")

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)

        self.main_frame()
        self.mainloop()

    def main_frame(self):
        # create sidebar frame with widgets

        self.sidebar_frame = customtkinter.CTkFrame(self, width=140, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = customtkinter.CTkLabel(
            self.sidebar_frame,
            text="Robutler",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.sidebar_button_1 = customtkinter.CTkButton(
            self.sidebar_frame, text="Talk to", command=self.talking_button_event
        )
        self.sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)

        self.appearance_mode_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Appearance Mode:", anchor="w"
        )
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            self.sidebar_frame,
            values=["Light", "Dark", "System"],
            command=self.change_appearance_mode_event,
        )
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="UI Scaling:", anchor="w"
        )
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(
            self.sidebar_frame,
            values=["80%", "90%", "100%", "110%", "120%"],
            command=self.change_scaling_event,
        )
        self.scaling_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 20))

        # create main entry and button

        # create textbox

        # create tabview
        self.tabview = customtkinter.CTkTabview(self, width=250)
        self.tabview.grid(row=0, column=2, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.tabview.add("Action")

        self.tabview.tab("Action").grid_columnconfigure(
            0, weight=1
        )  # configure grid of individual tabs

        self.optionmenu_1 = customtkinter.CTkOptionMenu(
            self.tabview.tab("Action"),
            dynamic_resizing=False,
            values=["Kitchen", "main_room", "Office", "small_room", "Living_room"],
        )
        self.optionmenu_1.grid(row=0, column=0, padx=20, pady=(20, 10))
        self.combobox_1 = customtkinter.CTkComboBox(
            self.tabview.tab("Action"),
            values=[
                "Move",
                "Look for sphere",
                "Is there anyone",
                "Is Suitcase There",
                "Is the table clean",
            ],
        )
        self.combobox_1.grid(row=1, column=0, padx=20, pady=(10, 10))

        self.button_1 = customtkinter.CTkButton(
            self.tabview.tab("Action"), text="Submit", command=self.button_callback
        )
        self.button_1.grid(row=2, column=0, padx=20, pady=(10, 10))

        # create slider and progressbar frame
        self.slider_progressbar_frame = customtkinter.CTkFrame(
            self, fg_color="transparent"
        )

        self.slider_progressbar_frame.grid(
            row=1, column=1, columnspan=2, padx=(20, 0), pady=(20, 0), sticky="nsew"
        )

        self.slider_progressbar_frame.grid_columnconfigure(0, weight=1)
        self.slider_progressbar_frame.grid_rowconfigure(4, weight=1)

        self.seg_button_1 = customtkinter.CTkSegmentedButton(
            self.slider_progressbar_frame, command=self.move_callback
        )
        self.seg_button_1.grid(
            row=4, column=0, padx=(20, 10), pady=(10, 10), sticky="ew"
        )
        self.progressbar_1 = customtkinter.CTkProgressBar(self.slider_progressbar_frame)
        self.progressbar_1.grid(
            row=1, column=0, padx=(20, 10), pady=(10, 10), sticky="ew"
        )
        self.progressbar_2 = customtkinter.CTkProgressBar(self.slider_progressbar_frame)
        self.progressbar_2.grid(
            row=2, column=0, padx=(20, 10), pady=(10, 10), sticky="ew"
        )
        self.slider_1 = customtkinter.CTkSlider(
            self.slider_progressbar_frame,
            from_=0,
            to=1,
            number_of_steps=5,
        )
        self.slider_1.grid(row=3, column=0, padx=(20, 10), pady=(10, 10), sticky="ew")

        self.slider_2 = customtkinter.CTkSlider(
            self.slider_progressbar_frame, orientation="vertical"
        )
        self.slider_2.grid(
            row=0, column=1, rowspan=5, padx=(10, 10), pady=(10, 10), sticky="ns"
        )
        self.progressbar_3 = customtkinter.CTkProgressBar(
            self.slider_progressbar_frame, orientation="vertical"
        )
        self.progressbar_3.grid(
            row=0, column=2, rowspan=5, padx=(10, 20), pady=(10, 10), sticky="ns"
        )

        self.combobox_1.grid(row=1, column=0, padx=20, pady=(10, 10))

        self.seg_button_1.configure(
            values=["Forward", "Backwards", "Rotate Left", "Rotate Right", "Stop"]
        )

        # set default values

        self.appearance_mode_optionemenu.set("Dark")
        self.scaling_optionemenu.set("100%")
        self.optionmenu_1.set("House Division")
        self.combobox_1.set("Robot Action")
        self.slider_1.configure(command=self.print_slider1)
        self.slider_2.configure(command=self.print_slider2)

        self.progressbar_1.configure(mode="indeterminnate")
        self.progressbar_1.start()

    def print_slider1(self, value):
        self.bus.post(event("gui", ("velh:" + str(value))))

        self.progressbar_2.set(value)

    def print_slider2(self, value):
        self.bus.post(event("gui", ("vel:" + str(value))))
        self.progressbar_3.set(value)

    def print_progressbar(self):
        print()

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100

    def talking_button_event(self):
        self.bus.post(event("gui", ("talking:0")))

    def button_callback(self):
        data = str(self.combobox_1.get() + ":" + self.optionmenu_1.get())

        self.bus.post(event("gui", data))

    def move_callback(self, value):
        self.bus.post(event("gui", ("move:" + value)))
