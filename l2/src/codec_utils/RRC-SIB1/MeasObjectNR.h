/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasObjectNR_H_
#define	_MeasObjectNR_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include "SubcarrierSpacing.h"
#include "ReferenceSignalConfig.h"
#include <NativeInteger.h>
#include "Q-OffsetRangeList.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "FreqBandIndicatorNR.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>
#include "MeasGapId-r17.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasObjectNR__ext1__measCycleSCell {
	MeasObjectNR__ext1__measCycleSCell_sf160	= 0,
	MeasObjectNR__ext1__measCycleSCell_sf256	= 1,
	MeasObjectNR__ext1__measCycleSCell_sf320	= 2,
	MeasObjectNR__ext1__measCycleSCell_sf512	= 3,
	MeasObjectNR__ext1__measCycleSCell_sf640	= 4,
	MeasObjectNR__ext1__measCycleSCell_sf1024	= 5,
	MeasObjectNR__ext1__measCycleSCell_sf1280	= 6
} e_MeasObjectNR__ext1__measCycleSCell;
typedef enum MeasObjectNR__ext3__measCyclePSCell_r17 {
	MeasObjectNR__ext3__measCyclePSCell_r17_ms160	= 0,
	MeasObjectNR__ext3__measCyclePSCell_r17_ms256	= 1,
	MeasObjectNR__ext3__measCyclePSCell_r17_ms320	= 2,
	MeasObjectNR__ext3__measCyclePSCell_r17_ms512	= 3,
	MeasObjectNR__ext3__measCyclePSCell_r17_ms640	= 4,
	MeasObjectNR__ext3__measCyclePSCell_r17_ms1024	= 5,
	MeasObjectNR__ext3__measCyclePSCell_r17_ms1280	= 6,
	MeasObjectNR__ext3__measCyclePSCell_r17_spare1	= 7
} e_MeasObjectNR__ext3__measCyclePSCell_r17;

/* Forward declarations */
struct SSB_MTC;
struct SSB_MTC2;
struct ThresholdNR;
struct PCI_List;
struct CellsToAddModList;
struct PCI_RangeIndexList;
struct PCI_RangeElement;
struct SSB_MTC3List_r16;
struct SetupRelease_RMTC_Config_r16;
struct SetupRelease_T312_r16;
struct SSB_MTC4List_r17;
struct CellsToAddModListExt_v1710;

/* MeasObjectNR */
typedef struct MeasObjectNR {
	ARFCN_ValueNR_t	*ssbFrequency;	/* OPTIONAL */
	SubcarrierSpacing_t	*ssbSubcarrierSpacing;	/* OPTIONAL */
	struct SSB_MTC	*smtc1;	/* OPTIONAL */
	struct SSB_MTC2	*smtc2;	/* OPTIONAL */
	ARFCN_ValueNR_t	*refFreqCSI_RS;	/* OPTIONAL */
	ReferenceSignalConfig_t	 referenceSignalConfig;
	struct ThresholdNR	*absThreshSS_BlocksConsolidation;	/* OPTIONAL */
	struct ThresholdNR	*absThreshCSI_RS_Consolidation;	/* OPTIONAL */
	long	*nrofSS_BlocksToAverage;	/* OPTIONAL */
	long	*nrofCSI_RS_ResourcesToAverage;	/* OPTIONAL */
	long	 quantityConfigIndex;
	Q_OffsetRangeList_t	 offsetMO;
	struct PCI_List	*cellsToRemoveList;	/* OPTIONAL */
	struct CellsToAddModList	*cellsToAddModList;	/* OPTIONAL */
	struct PCI_RangeIndexList	*excludedCellsToRemoveList;	/* OPTIONAL */
	struct MeasObjectNR__excludedCellsToAddModList {
		A_SEQUENCE_OF(struct PCI_RangeElement) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *excludedCellsToAddModList;
	struct PCI_RangeIndexList	*allowedCellsToRemoveList;	/* OPTIONAL */
	struct MeasObjectNR__allowedCellsToAddModList {
		A_SEQUENCE_OF(struct PCI_RangeElement) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *allowedCellsToAddModList;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct MeasObjectNR__ext1 {
		FreqBandIndicatorNR_t	*freqBandIndicatorNR;	/* OPTIONAL */
		long	*measCycleSCell;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct MeasObjectNR__ext2 {
		struct SSB_MTC3List_r16	*smtc3list_r16;	/* OPTIONAL */
		struct SetupRelease_RMTC_Config_r16	*rmtc_Config_r16;	/* OPTIONAL */
		struct SetupRelease_T312_r16	*t312_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct MeasObjectNR__ext3 {
		MeasGapId_r17_t	*associatedMeasGapSSB_r17;	/* OPTIONAL */
		MeasGapId_r17_t	*associatedMeasGapCSIRS_r17;	/* OPTIONAL */
		struct SSB_MTC4List_r17	*smtc4list_r17;	/* OPTIONAL */
		long	*measCyclePSCell_r17;	/* OPTIONAL */
		struct CellsToAddModListExt_v1710	*cellsToAddModListExt_v1710;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	struct MeasObjectNR__ext4 {
		MeasGapId_r17_t	*associatedMeasGapSSB2_v1720;	/* OPTIONAL */
		MeasGapId_r17_t	*associatedMeasGapCSIRS2_v1720;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext4;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasObjectNR_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_measCycleSCell_25;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_measCyclePSCell_r17_41;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MeasObjectNR;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasObjectNR_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasObjectNR_1[22];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SSB-MTC.h"
#include "SSB-MTC2.h"
#include "ThresholdNR.h"
#include "PCI-List.h"
#include "CellsToAddModList.h"
#include "PCI-RangeIndexList.h"
#include "PCI-RangeElement.h"
#include "SSB-MTC3List-r16.h"
#include "SetupRelease.h"
#include "SSB-MTC4List-r17.h"
#include "CellsToAddModListExt-v1710.h"

#endif	/* _MeasObjectNR_H_ */
#include <asn_internal.h>
