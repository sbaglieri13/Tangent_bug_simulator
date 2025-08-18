# Tangent Bug 

Questo progetto mostra l'esecuzione dell’algoritmo **Tangent Bug** in un ambiente 2D con ostacoli rettangolari generati in modo casuale.  
Viene generata una **simulazione animata** del robot e un **grafico** con lo *scan* del sensore (angolo θ → distanza).

---


## Descrizione dell’algoritmo Tangent Bug

Il Tangent Bug alterna due comportamenti principali:

1. **Move-to-goal**
   - Esegue una scansione a 360° e calcola i **punti di discontinuità** $O_k$.
   - Se il goal è visibile, si muove verso il goal.
   - Altrimenti si dirige verso il punto $O_k$ che minimizza l’euristica
     $$ h_k(q) = d(q, O_k) + d(O_k, \text{goal}) \, . $$
   - Passa al *boundary following* quando la minima $h_k(q)$ inizia ad aumentare.

2. **Boundary-following**
   - Segue il bordo dell’ostacolo mantenendo una distanza desiderata.
   - Confronta:
     - $d_{\text{reach}}$: distanza “diretta” verso il goal (se visibile) o un surrogato basato sui punti colpiti dal LIDAR;
     - $d_{\text{followed}}$: distanza dal goal del miglior punto “candidato” visto lungo il bordo.
   - Ritorna a *move-to-goal* quando **$d_{\text{reach}} < d_{\text{followed}}$** (oppure quando il goal viene raggiunto).


---

## Struttura del progetto

```
.
├─ tb/
│  ├─ __init__.py
│  ├─ sim.py          # entrypoint: run_simulation()
│  ├─ robot.py        # logica del robot 
│  ├─ geometry.py     # ostacoli + funzioni geometriche 
│  ├─ plotting.py     # utility di disegno 
│  └─ utils.py        # costanti/aiuti (seed, formattazioni)
├─ main.py          
├─ requirements.txt
└─ README.md
```

---

## Requisiti

- Python **3.9+**
- Dipendenze Python:
  - `matplotlib`
  - `numpy`
  - `pillow` 

Installa tutto con:

```bash
pip install -r requirements.txt
```

---

## Esecuzione

```bash
python main.py
```
---

## Output dell’esecuzione

1. **Finestra di simulazione**  
   - Start (blu), Goal (verde)  
   - Robot che **cambia colore** a seconda del comportamento  
     *(blu = move-to-goal, arancione = boundary-following)*  
   - Path blu, discontinuità segnate con “X” rosse  
   - Badge in basso a destra con il **comportamento corrente**

2. **Grafico θ → distanza**  
   - Asse X: **θ** in gradi (0–360)  
   - Asse Y: **distanza** misurata dal LIDAR  

3. **File salvati automaticamente nella cartella "media"**
   - `tangent_bug_sim.gif` – animazione della simulazione  
   - `plot.png` – grafico finale θ → distanza

> I nomi file e i parametri (es. FPS della GIF) si possono modificare in `tb/sim.py` dentro `run_simulation()`.

---

## Demo

**Simulazione percorso**
![Demo – Tangent Bug (GIF)](media/tangent_bug_sim.gif)  


**Grafico θ → distanza**
![Plot (θ vs distanza)](media/plot.png)  
---

## Parametri utili (modificabili in `run_simulation()`)

- Mappa/ostacoli: `num_obstacles`, `min_obs_size`, `max_obs_size`, `max_attempts`
- Robot/sensore: `robot_radius`, `sensing_range`
- Export: `gif_path`, `png_path`, `gif_fps`



