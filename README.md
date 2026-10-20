# PHOENIX-ATMOS-XP

**Atmospheric Technology for Mobility, Optimisation and Sustainability — Experimental Propulsion**

An installable, offline-capable engineering platform for a wind-primary, metamaterial-assisted
automobile with no battery, no fuel and no traction motor. It contains 26 coupled engineering suites —
from live atmosphere, CAD and meshing, CFD, aeroelastic wings, FSI and composite FEA, through photonic
collectors, thermal metamaterials, NiTi thermomechanics and the direct-rim wheel-hub drive, to vehicle
dynamics, braking, acoustics, manufacturing, safety, verification & validation, optimisation, economics
and sustainability.

**Live app:** https://samuelakosaonyejekwe.github.io/atmosxp/

## What it does
- **26 suites**, each with Automatic / Guided / Expert / Import-experimental-data input modes, provenance on
  every value, locked inputs, governing-equation documentation, initial & boundary conditions, calibration,
  verification and validation plans, explained results, verdicts and recommendations.
- **Coupled engines** — atmosphere → aerodynamics → aeroelastic wing → vehicle dynamics; radiation →
  photonics → thermal → NiTi → hub; hub → tyres → road load; buffer → switching → propulsion; all outputs →
  fatigue, reliability, manufacturing and life-cycle assessment — governed by one energy/exergy balance.
- **Mandatory feasibility tests** (aerodynamic, thermal, mechanical, integrated vehicle) across day, night,
  windy, calm, gusting, humid, cloudy and changing conditions. Failures are shown with causes and fixes.
- **Live global data** fetched directly by each user's device: Open-Meteo (forecast, ERA5 reanalysis,
  geocoding, elevation, air quality), NASA POWER climatology, OpenStreetMap Nominatim, NWS alerts, USGS
  hazards, OpenAlex / Crossref / Europe PMC literature & prior art, World Bank indicators, ECB/Frankfurter FX,
  Our World in Data grid carbon intensity, UK grid carbon, PubChem and Wikidata.
- **CAD & meshing** — parametric vehicle and hub assemblies, NACA/CST airfoils, dimensioned third-angle
  drawings, mesh generation with quality metrics, mesh-sensitivity studies (GCI) and automatic mesh selection.
- **Exports** — STEP (faceted B-rep), STL, OBJ, PLY, glTF/GLB, DXF, SVG, airfoil DAT/CSV, .msh 2.2, VTK, .su2,
  INP keyword decks, BDF bulk-data decks, CSV/JSON, PNG/SVG plots, research evidence packages and full project ZIPs.
- **Data portal** — CSV/TSV/TXT/DAT, JSON/GeoJSON, XLSX/XLS/ODS, XML/GPX/KML, STL, OBJ, MSH, VTK.
- **Materials library** with datasheets, uncertainty and sources; component-by-component assignment.
- **Component dossier** — architecture rationale, fabrication, drawings, experiment replication, operation.

## Installation & offline use
Open the live app and press **⤓ Install app** or use the browser's **Share → Add to Home Screen**.
After the first visit everything runs offline, including in airplane mode. The *Install* page also offers a
self-hostable offline copy (ZIP) of the whole application.

## Redundancy
The application is a static, server-less build and can be served from any static host. Ready-made
configurations for several static hosts are included (`netlify.toml`, `vercel.json`, `_headers`,
`.gitlab-ci.yml`; a `pages` branch also works).
Installed copies keep working when every host is unavailable.

## Local development
```
node tools/build.mjs        # regenerate the service-worker precache manifest
# serve the folder with any static file server, then open it in a browser
```
No other build step or dependency is required. Vendored 3D and spreadsheet libraries are in `vendor/` with their licences.

## Scientific rules
Simulated predictions are not physical validation. The software never fabricates measurements, compares
materials only under identical conditions, resolves aerodynamic forces into body/ground frames (lift is not
thrust), and rejects any propulsion result that exceeds available environmental exergy plus finite stored energy.
