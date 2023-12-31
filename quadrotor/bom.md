# Bill of Materials

The video tutorial to build the drone is accessible at this [link](https://youtu.be/xvOS7IEFxlU).

**[UPDATE]** Please note the following 4 differences between the video and the current BOM/STLs.

- A Shorter WiFi antenna is used for the NX, it can be placed on the NX fan and under the top case.
- The camera holding now has room for the ribbon cable, and a small hole below for the cable to pass through.
- A shielded cable is suggested for the serial connection between the NX and the flight controller.
- A serial holder is provided among the 3D files, to hold the serial cable in place.

## 3D Prints

| **COMPONENT**                                  |   **FILE**               |
| ---------------------------------------------- | ------------------------ |
| Camera front case With FPV holding             | [arducam_and_fpv_case.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/arducam_and_fpv_case.stl) |
| Camera front case Without FPV holding          | [arducam_case.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/arducam_case.stl)                 |
| NX Bottom Plate                                | [nx_bottom.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/nx_bottom.stl)                       |
| NX Top 30°                                     | [nx_top30.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/nx_top30.stl)                         |
| NX Top 40°                                     | [nx_top40.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/nx_top40.stl)                         |
| NX Top 50°                                     | [nx_top50.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/nx_top50.stl)                         |
| BEC Support                                    | [bec_support.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/bec_support.stl)                   |
| 6S Battery Cage                                | [battery-cage.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/battery-cage.stl)                 |
| Serial Holder                                  | [serial_holder.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/serial_holder.stl)               |
| MoCap Marker Arm                               | [mocap_arm.stl](https://github.com/Drone-Racing/drone-racing-dataset/blob/main/quadrotor/3d_print/mocap_arm.stl)                       |

## Quadrotor Electronics

| **COMPONENT**     |  **NAME**                             | **LINK**                                                                                                                                                           | **Price (USD)** | **QTY** | **Total (USD)** |
| ----------------- | ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------- | ------- | --------------- |
| Motors            | Tmotor F60PROV 2020KV                 | [T-Motor](https://store.tmotor.com/goods-1217-F60PROV.html)                                                                                                        | 26.9            | 4       | 107.6           |
| ESC               | Tmotor F55A PROⅡ 6S 4IN1              | [T-Motor](https://store.tmotor.com/goods-821-F55A_PRO_6S_4IN1_.html)                                                                                               | 93.9            | 1       | 93.9            |
| Flight Controller | Holybro Kakute H7 V1.3                | [Holybro](https://holybro.com/collections/autopilot-flight-controllers/products/kakute-h7)                                                                         | 96.59           | 1       | 96.59           | 
| Receiver          | TBS CROSSFIRE NANO RX (SE) LONG RANGE | [TBS](https://www.team-blacksheep.com/products/prod:crossfire_nano_se)                                                                                             | 29.95           | 1       | 29.95           |  

## Autonomous Module

| **COMPONENT**       | **NAME**                                                                                          | **LINK**                                                                                                                   | **Price (USD)** | **QTY** | **Total (USD)** |
| ------------------- | ------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- | -------------- | -------- | --------------- |
| Nvidia NX Orin      | Jetson Orin NX 16GB Module 900-13767-0000-000                                                     | [Arrow](https://www.arrow.com/en/products/900-13767-0000-000/nvidia)                                                       | 579            | 1        | 579             |
| Carrier Board       | Seed Studio A203 (Version 2) Carrier Board for Jetson                                             | [Seeedstudio](https://www.seeedstudio.com/A203-Carrier-Board-for-Jetson-Nano-Xavier-NX-V2-p-5214.html)                     | 179            | 1        | 179             |
| Heatsink            | Seed Studio Aluminum Heatsink with Fan for Jetson Orin NX                                         | [Seeedstudio](https://www.seeedstudio.com/Aluminum-Heatsink-with-Fan-for-Jetson-Orin-NX-Orin-Nano-Xavier-NX-Module-p-5633.html) | 19.9      | 1        | 19.9            |
| BEC                 | Matek BEC12S-PRO, 9-55V TO 5V/8V/12V-5A                                                           | [Mateksys](http://www.mateksys.com/?portfolio=bec12s-pro)                                                                  | 21.99          | 1        | 21.99           |
| SSD                 | Sabrent 1TB Rocket NVMe PCIe M.2 2242                                                             | [Sabrent](https://sabrent.com/products/sb-1342-1tb?_pos=3&_sid=722ff7792&_ss=r)                                            | 119.99         | 1        | 119.99          |
| WiFi adapter card   | Wi-Fi 6E AX210NGW Wireless WiFi Card BT5.2 M.2                                                    | [Google](https://www.google.com/search?q=Wi-Fi+6E+AX210NGW+Wireless+WiFi+Card+BT5.2+M.2)                                   | 25.99          | 1        | 25.99           |
| WiFi Antenna        | Wi-Fi 6E Flex Cabled Balanced Antenna 50.00mm Cable Length, Compatible with I-PEX MHF4 Connectors | [Molex](https://www.molex.com/molex/products/part-detail/antennas/1461531050)                                              | 2.57           | 1        | 2.57            |
| Camera              | Arducam 8MP IMX219 Wide Angle Camera Module                                                       | [Arducam](https://www.arducam.com/product/b0179-arducam-imx219-wide-angle-camera-module-for-nvidia-jetson-nano/)           | 19.99          | 1        | 19.99           |

## FPV System (for human-piloted flights only)

| **COMPONENT**     |  **NAME**                             | **LINK**                                                                                                                                                           | **Price (USD)** | **QTY** | **Total (USD)** |
| ----------------- | ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------- |  ------ | --------------- |
| FPV Camera        | Foxeer T-Rex Mini 1500TVL             | [Foxeer](https://www.foxeer.com/foxeer-t-rex-mini-1500tvl-6ms-low-latency-super-wdr-fpv-camera-g-314)                                                              | 44.9            | 1       | 44.9            |
| VTX           | TBS UNIFY PRO 5G8 HV (SMA)                        | [TBS](https://www.team-blacksheep.com/products/prod:unify_pro_hv_sma)                                                | 49.95             | 1       | 49.95             |
| FPV Antenna           | TBS TRIUMPH PRO (SMA)                        | [TBS](https://www.team-blacksheep.com/products/prod:triumph_pro_sma)                                                | 19.95             | 1       | 19.95             |
| Goggles           | Fat Shark HDO2                        | [Fat Shark](https://pyrodrone.com/collections/fatshark-products/products/fatshark-dominator-hdo-2-oled-fpv-goggles)                                                | 499             | 1       | 499             |

______

## Frame and Fasteners

| **COMPONENT**           | **NAME**                                                           | **LINK**                                                                                            | **Price (USD)** | **QTY** | **Set PCS** | **PCS Used** | **Total (USD)** |  **Notes** |
| ----------------------- | ------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------- | --------------- |  ------ | ----------- | ------------ | --------------- |  --------- |
| Frame                   | Pyrodrone HYPERLITE FLOSS 3.0 RACE FRAME 5"                        | [Pyrodrone](https://pyrodrone.com/products/hyperlite-floss-3-0-race-frame)                          | 44.99           | 1       |             |              | 44.99           |            |
| Rods Bar                | M3 x 30mm Stainless Steel Fully Threaded Rods Bar                  | [Amazon](https://www.amazon.com/uxcell-0-5mm-Stainless-Threaded-Silver/dp/B01MDOFYCF)               | 11.49           | 1       | 20          | 4            | 11.49           |            |
| Standoff                | M3 x 60 mm female standoff                                         | [Google](https://www.google.com/search?q=m3+60+mm+female+standoff)                                  | 13              | 1       |             | 4            | 13              |  [The ones used in the video are from rjxhobby 50pcs](https://www.rjxhobby.com/Accessories/alum-standoff-spacer/rjx-50pcs-m3-aluminum-female-threaded-sleeve-bushing-hex-standoff-pillars-connector-nuts) |
| Nylon bolts             | M3 Metric Black Nylon Hex Nut, Hexagonal                           | [Amazon](https://www.amazon.com/Electronics-Salon-100pcs-Metric-Black-Hexagonal/dp/B0129BDGW6)      | 4.8             | 1        | 100        | 8            | 4.8             |            |
| Screws                  | M3 x 10mm Hex Button Head screws                                   | [Amazon](https://www.amazon.com/HELIFOUNER-Pieces-Button-Washers-Stainless/dp/B097Y4W8RF)           | 12.99           | 1        | 30         | 8            | 12.99           |  The set in the link contains also unused sizes  |
| Screws                  | M3 x 25mm Hex Button Head screws                                   | [Amazon](https://www.amazon.com/HELIFOUNER-Pieces-Button-Washers-Stainless/dp/B097Y4W8RF)           | 12.99           | 1        | 20         | 4            | 12.99           |  The set in the link contains also unused sizes  |
| Self tapping Screws     | M2 x 10mm self tapping screws                                      | [Amazon](https://www.amazon.com/Nickel-Plated-Tapping-Assortment-Drilling-Storage/dp/B08FQWXCKG)    | 11.99           | 1        | 100        | 4            | 11.99           |  The set in the link contains also unused sizes  |
| Battery strap           | 20 x 250 mm battery strap                                          | [Google](https://www.google.com/search?q=20+x+250+mm+battery+strap)                                 | 5               | 1        |            |              | 5               |            |
| Propellers              | Tmotor T5147                                                       | [T-Motor](https://store.tmotor.com/goods-820-T5147+2PairsBag.html)                                  | 3.19            | 10       |            |              | 31.9            |            |

## Assembly Elements

| **COMPONENT**     | **NAME**                                                          | **LINK**                                                                                        | **Price (USD)** | **QTY** | **Set PCS** | **PCS Used**  | **Total (USD)** |
| ----------------- | ----------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- | --------------- | ------- | ------------ | ------------ | --------------- |
| XT30 M+F          | XT30 Connectors Male and Female Connectors Plugs with heat shrink | [Amazon](https://www.amazon.com/Hobbypark-Bullet-Connectors-Female-Battery/dp/B07PN8WVGT)       | 8.98            | 1       | 10           | 1            | 8.98            |
| Cables            | 16AWG silicone cable                                              | [Google](https://www.google.com/search?q=16awg+cable+silicone)                                  | 28              | 1       |              |              | 28              |
| Shielded Cable           |  RVVP Shielded Cable 3 pin 0.5mm Copper Wire                                   | [Aliexpress](https://www.aliexpress.us/item/32976787590.html)                              | 13.84            | 1       |              |              | 13.84            |
| Heat shrink tube  | Heat shrink tube 30mm                                             | [Google](https://www.google.com/search?q=heat+shrink+tube+30mm)                                 | 6.49            | 1       |              |              | 6.49            |
| Double sided tape | GORILLA HEAVY DUTY MOUNTING TAPE 1"X60"-Black                     | [Amazon](https://www.amazon.com/Gorilla-Double-Sided-Heavy-Mounting-Black/dp/B07PYG86FC)        | 11.74           | 1       |              |              | 11.74           |
| Tape              | Electrical tape                                                   | [Google](https://www.google.com/search?q=electrical+tape)                                       | 4               | 1       |              |              | 4               |
| Zipties           | 10 cm zipties                                                     | [Google](https://www.google.com/search?q=10+cm+zipties)                                         | 10              | 1       |              |              | 10              |

## Battery and Charger

| **COMPONENT** | **NAME**                                                                          | **LINK**                                                                                                              | **Price (USD)** | **QTY** | **Total (USD)** |
| ------------- | --------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- | --------------- | ------- | ----------- |
| Battery       | Tattu R-Line Version 5.0 1400mAh 22.2V 150C 6S1P Lipo Battery Pack With XT60 Plug | [Genstattu](https://genstattu.com/tattu-r-line-version-5-0-1400mah-22-2v-150c-6s1p-lipo-battery-pack-with-xt60-plug/) | 40.99           | 5       | 204.95      |
| Charger       | HOTA D6 Pro Charger                                                               | [iFlight-RC](https://shop.iflight-rc.com/HOTA-D6-Pro-AC200W-DC650W-15A-1-6S-Dual-Channel-Charger-Pro1796)             | 118.99          | 1       | 118.99      |

## Radio Controller

| **COMPONENT** | **NAME**                                            | **LINK**                                                                                                                             | **Price (USD)** | **QTY** | **Total (USD)** | **Notes**                                                                                                               |
| ------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | --------------- | ------- | --------------- |  ---------------------------------------------------------------------------------------------------------------------- |
| Controller    | RADIOMASTER TX16S Mark II Radio Controller (Mode 2) | [RadiomasterRC](https://www.radiomasterrc.com/collections/tx16s-mkii/products/tx16s-mark-ii-radio-controller?variant=42817741291751) | 199.99          | 1       | 199.99          |                                                                                                                         |
| Transmitter   | TBS CROSSFIRE MICRO TX V2                           | [TBS](https://www.team-blacksheep.com/products/prod:crossfire_micro_tx)                                                              | 69.95           | 1       | 69.95           | [One could buy the kit with the receiver](https://www.radiomasterrc.com/products/tbs-crossfire-micro-tx-v2-starter-set) |

## Tools and Accessories (optional)

| **COMPONENT**      | **NAME**                                                                    | **LINK**                                                                                                   | **Price (USD)** | **QTY** | **Total (USD)** |
| ------------------ | --------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- | --------------- | ------- | --------------- |
| Precision Tweezers | 6PCS Precision Tweezers Set                                                 | [Amazon](https://www.amazon.com/Precision-Anti-Static-Electronics-Laboratory-Jewelry-Making/dp/B07RNVXXV1) | 7.99            | 1       | 7.99            |
| iFixit kit         | iFixit Manta Driver Kit                                                     | [Amazon](https://www.amazon.com/iFixit-Manta-Driver-Kit-Piece/dp/B07BMM74FD)                               | 69.99           | 1       | 69.99           |
| Hex Driver         | 7pcs Hex Allen Screwdriver Kit 0.9mm 1.27mm 1.3mm 1.5mm 2.0mm 2.5mm 3.0mm   | [Amazon](https://www.amazon.com/driver-Screwdriver-Helicopter-Precision-Engineering/dp/B07V8SNSNV)         | 17.99           | 1       | 17.99           |
| Pliers             | 6PCS Mini Pliers Set                                                        | [Amazon](https://www.amazon.com/HAUTMEC-leverage-Linesman-Diagonal-Cutters/dp/B0BQVQZFK9)                  | 26.99           | 1       | 26.99           |
| Scissors           | Scissors                                                                    | [Amazon](https://www.amazon.com/Fiskars-01-004761J-Softgrip-Scissors-Stainless/dp/B002YIP97K)              | 6.48            | 1       | 6.48            |
| Wire Cutter        | VCELINK Small Wire Cutter Spring-loaded                                     | [Amazon](https://www.amazon.com/VCELINK-Spring-loaded-Precision-Diagonal-Electronics/dp/B09SL2TCH7)        | 6.88            | 1       | 6.88            |
| Wire Stripper      | WGGE WG-015 Professional 8-inch Wire Stripper                               | [Amazon](https://www.amazon.com/WGGE-Professional-crimping-Multi-Tool-Multi-Function/dp/B073YG65N2)        | 8.59            | 1       | 8.59            |
| Soldering Iron     | UY CHAN Original TS101 Soldering Pen                                        | [Amazon](https://www.amazon.com/UY-CHAN-Programmable-Pocket-size-Acceleration/dp/B01MDTO6X7)               | 76.99           | 1       | 76.99           |
| Soldering Iron tip | UY CHAN Original Soldering Iron Tip C4                                      | [Amazon](https://www.amazon.com/UY-CHAN-Original-Soldering-Replacement/dp/B06XYS34N4)                      | 15.98           | 1       | 15.98           |
| Solder             | SONEAK 60/40 Tin Lead Solder With Rosin Core For Electrical Soldering 1.0mm | [Amazon](https://www.amazon.com/SONEAK-Solder-Rosin-Electrical-Soldering/dp/B084RZF23H)                    | 7.99            | 1       | 7.99            |
| Heat gun           | Wagner Spraytech 2417344 HT1000 Heat Gun Kit                                | [Amazon](https://www.amazon.com/Wagner-Spraytech-2417344-Included-Settings/dp/B08RHDWTW1)                  | 27.99           | 1       | 27.99           |
| Battery checker           | Smart battery checker                                | [TBS](https://www.team-blacksheep.com/products/product:1809)                  | 39.99           | 1       | 39.99           |
| Battery bag           | Lipo safe battery bag                                | [Google](https://www.google.com/search?q=lipo+safe+battery+bag)                  | 14.99           | 1       | 14.99           |