bl_info = {
    "name": "Lidar Trajectory Smoother",
    "author": "Asystent AI",
    "version": (1, 0),
    "blender": (4, 0, 0), # Działa również poprawnie z 5.1
    "location": "View3D > N-Panel > Lidar",
    "description": "Wygładza pliki trajektorii z użyciem filtru Savitzky'ego-Golaya",
    "category": "3D View",
}

import bpy
import os
import sys
import subprocess
import numpy as np

# -------------------------------------------------------------------
# FUNKCJA SPRAWDZAJĄCA ZALEŻNOŚCI (SCIPY)
# -------------------------------------------------------------------
def is_scipy_installed():
    try:
        import scipy
        return True
    except ImportError:
        return False

# -------------------------------------------------------------------
# OPERATOR DO INSTALACJI SCIPY (JEŚLI BRAKUJE)
# -------------------------------------------------------------------
class LIDAR_OT_install_scipy(bpy.types.Operator):
    bl_idname = "lidar.install_scipy"
    bl_label = "Zainstaluj brakującą bibliotekę (SciPy)"
    bl_description = "Pobiera i instaluje bibliotekę SciPy przy użyciu wbudowanego w Blendera pip"

    def execute(self, context):
        try:
            self.report({'INFO'}, "Rozpoczęto instalację SciPy. Zawieś oko na oknie konsoli, to może chwilę potrwać...")
            
            # Uzyskanie ścieżki do pythona używanego przez Blendera
            python_exe = sys.executable
            
            # Uruchomienie pip install
            subprocess.check_call([python_exe, "-m", "pip", "install", "scipy"])
            
            self.report({'INFO'}, "Instalacja SciPy zakończona sukcesem! Możesz teraz wygładzić trajektorię.")
            
        except subprocess.CalledProcessError as e:
            self.report({'ERROR'}, "Błąd instalacji. Spróbuj uruchomić Blendera jako Administrator.")
            print(f"Błąd instalacji pip: {e}")
        except Exception as e:
            self.report({'ERROR'}, f"Nieoczekiwany błąd: {str(e)}")
            
        return {'FINISHED'}

# -------------------------------------------------------------------
# GŁÓWNY OPERATOR WYGŁADZAJĄCY TRAJEKTORIĘ
# -------------------------------------------------------------------
class LIDAR_OT_smooth_trajectory(bpy.types.Operator):
    bl_idname = "lidar.smooth_trajectory"
    bl_label = "Stabilizuj Trajektorię"
    bl_description = "Wczytuje plik, filtruje szumy pozycji i rotacji, i zapisuje plik z sufiksem _smoothed"

    def execute(self, context):
        props = context.scene.lidar_props
        
        # Pobranie ścieżki i konwersja ścieżki względnej (jeśli taka jest) na bezwzględną
        input_file = bpy.path.abspath(props.filepath)
        
        if not input_file or not os.path.exists(input_file):
            self.report({'ERROR'}, "Wybrano nieprawidłowy plik lub plik nie istnieje!")
            return {'CANCELLED'}
            
        try:
            from scipy.signal import savgol_filter
            from scipy.spatial.transform import Rotation as R
        except ImportError:
            self.report({'ERROR'}, "Moduł SciPy nadal nie jest dostępny. Zrestartuj Blendera lub zainstaluj ponownie.")
            return {'CANCELLED'}

        window_length = props.window_length
        polyorder = 3
        
        if window_length <= polyorder:
            self.report({'ERROR'}, "Długość okna musi być większa niż 3!")
            return {'CANCELLED'}

        try:
            self.report({'INFO'}, f"Przetwarzanie pliku: {os.path.basename(input_file)}")
            
            # Wczytanie danych
            data = np.loadtxt(input_file, comments='#')
            timestamps = data[:, 0]
            positions = data[:, 1:4]
            quaternions = data[:, 4:8]

            # 1. Wygładzanie pozycji X, Y, Z
            smoothed_positions = np.zeros_like(positions)
            for i in range(3):
                smoothed_positions[:, i] = savgol_filter(positions[:, i], window_length, polyorder)

            # 2. Wygładzanie rotacji (via wektory rotacji - axis angle)
            rotations = R.from_quat(quaternions)
            rot_vecs = rotations.as_rotvec()

            smoothed_rot_vecs = np.zeros_like(rot_vecs)
            for i in range(3):
                smoothed_rot_vecs[:, i] = savgol_filter(rot_vecs[:, i], window_length, polyorder)

            # Powrót do kwaternionów
            smoothed_rotations = R.from_rotvec(smoothed_rot_vecs)
            smoothed_quaternions = smoothed_rotations.as_quat()

            # 3. Zapis do pliku wynikowego
            output_file = os.path.splitext(input_file)[0] + "_smoothed.txt"
            output_data = np.hstack((timestamps.reshape(-1, 1), smoothed_positions, smoothed_quaternions))

            header = "# timestamp tx ty tz qx qy qz qw"
            np.savetxt(output_file, output_data, fmt="%.6f", header=header, comments="")

            self.report({'INFO'}, f"Sukces! Zapisano do: {os.path.basename(output_file)}")

        except Exception as e:
            self.report({'ERROR'}, f"Błąd w trakcie przeliczania: {str(e)}")
            print(f"Error details: {e}")
            return {'CANCELLED'}

        return {'FINISHED'}

# -------------------------------------------------------------------
# DEFINICJA WŁAŚCIWOŚCI (ZMIENNYCH) DLA UI
# -------------------------------------------------------------------
def update_window_length(self, context):
    """Wymusza, aby window_length zawsze było liczbą nieparzystą"""
    if self.window_length % 2 == 0:
        self.window_length += 1

class LidarProperties(bpy.types.PropertyGroup):
    filepath: bpy.props.StringProperty(
        name="Plik Trajektorii",
        description="Wybierz plik tekstowy z danymi trajektorii (.txt)",
        subtype='FILE_PATH'
    )
    
    window_length: bpy.props.IntProperty(
        name="Długość okna (Moc filtra)",
        description="Rozmiar okna filtra (musi być nieparzyste). Większe okno mocniej wygładza uskok",
        default=151,
        min=1,          # Twardy limit minimum
        max=10000,      # Twardy limit maksimum
        soft_min=51,    # Miękki limit na suwaku
        soft_max=1501,  # Miękki limit na suwaku
        step=50,        # Skok suwaka
        update=update_window_length
    )

# -------------------------------------------------------------------
# PANEL UI W N-PANELU
# -------------------------------------------------------------------
class LIDAR_PT_panel(bpy.types.Panel):
    bl_label = "Narzędzia Stabilizacji"
    bl_idname = "LIDAR_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Lidar' # Nazwa zakładki w N-Panelu

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        props = scene.lidar_props

        # Pola do wyboru i ustawień
        layout.prop(props, "filepath")
        layout.prop(props, "window_length")
        
        layout.separator()

        # Sprawdzenie obecności SciPy i modyfikacja interfejsu
        if is_scipy_installed():
            layout.operator("lidar.smooth_trajectory", icon='MOD_SMOOTH')
            
            layout.separator()
            box = layout.box()
            box.label(text="Informacja:", icon='INFO')
            box.label(text="Plik zostanie zapisany w tym samym")
            box.label(text="katalogu z przyrostkiem _smoothed")
        else:
            box = layout.box()
            box.label(text="Brak biblioteki SciPy!", icon='ERROR')
            box.label(text="Jest ona niezbędna do matematyki filtru.")
            layout.operator("lidar.install_scipy", icon='CONSOLE')

# -------------------------------------------------------------------
# REJESTRACJA DODATKU
# -------------------------------------------------------------------
classes = (
    LIDAR_OT_install_scipy,
    LIDAR_OT_smooth_trajectory,
    LidarProperties,
    LIDAR_PT_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    # Rejestracja grupy właściwości globalnie dla sceny
    bpy.types.Scene.lidar_props = bpy.props.PointerProperty(type=LidarProperties)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.lidar_props

# Jeśli skrypt jest odpalany bezpośrednio z edytora tekstu w Blenderze:
if __name__ == "__main__":
    register()