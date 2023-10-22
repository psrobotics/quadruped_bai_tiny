PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//13722972/455385/2.49/2/3/Connector

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c270_h180"
		(holeDiam 1.8)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 2.7) (shapeHeight 2.7))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 2.7) (shapeHeight 2.7))
	)
	(padStyleDef "s270_h180"
		(holeDiam 1.8)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 2.7) (shapeHeight 2.7))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 2.7) (shapeHeight 2.7))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "SHDR2W160P0X500_1X2_1020X520X1" (originalName "SHDR2W160P0X500_1X2_1020X520X1")
		(multiLayer
			(pad (padNum 1) (padStyleRef s270_h180) (pt 0, 0) (rotation 90))
			(pad (padNum 2) (padStyleRef c270_h180) (pt 5, 0) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.85 -2.85) (pt -2.85 2.85) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.85 2.85) (pt 7.85 2.85) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 7.85 2.85) (pt 7.85 -2.85) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 7.85 -2.85) (pt -2.85 -2.85) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.6 -2.6) (pt -2.6 2.6) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.6 2.6) (pt 7.6 2.6) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 7.6 2.6) (pt 7.6 -2.6) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 7.6 -2.6) (pt -2.6 -2.6) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0 -2.6) (pt 7.6 -2.6) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 7.6 -2.6) (pt 7.6 2.6) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 7.6 2.6) (pt -2.6 2.6) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.6 2.6) (pt -2.6 0) (width 0.2))
		)
	)
	(symbolDef "XT30UPB-M" (originalName "XT30UPB-M")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 600 mils 100 mils) (width 6 mils))
		(line (pt 600 mils 100 mils) (pt 600 mils -200 mils) (width 6 mils))
		(line (pt 600 mils -200 mils) (pt 200 mils -200 mils) (width 6 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 650 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 650 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "XT30UPB-M" (originalName "XT30UPB-M") (compHeader (numPins 2) (numParts 1) (refDesPrefix J)
		)
		(compPin "1" (pinName "1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "XT30UPB-M"))
		(attachedPattern (patternNum 1) (patternName "SHDR2W160P0X500_1X2_1020X520X1")
			(numPads 2)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
			)
		)
		(attr "Manufacturer_Name" "Amass")
		(attr "Manufacturer_Part_Number" "XT30UPB-M")
		(attr "Mouser Part Number" "")
		(attr "Mouser Price/Stock" "")
		(attr "Arrow Part Number" "")
		(attr "Arrow Price/Stock" "")
		(attr "Description" "Socket; DC supply; XT30; male; PIN: 2; on PCBs; THT; Colour: yellow")
		(attr "<Hyperlink>" "https://www.tme.eu/Document/4acc913878197f8c2e30d4b8cdc47230/XT30UPB%20SPEC.pdf")
		(attr "<Component Height>" "10.7")
		(attr "<STEP Filename>" "XT30UPB-M.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
