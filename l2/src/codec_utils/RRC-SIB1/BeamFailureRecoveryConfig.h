/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BeamFailureRecoveryConfig_H_
#define	_BeamFailureRecoveryConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include "RSRP-Range.h"
#include <NativeEnumerated.h>
#include "SearchSpaceId.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "SubcarrierSpacing.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BeamFailureRecoveryConfig__ssb_perRACH_Occasion {
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_oneEighth	= 0,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_oneFourth	= 1,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_oneHalf	= 2,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_one	= 3,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_two	= 4,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_four	= 5,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_eight	= 6,
	BeamFailureRecoveryConfig__ssb_perRACH_Occasion_sixteen	= 7
} e_BeamFailureRecoveryConfig__ssb_perRACH_Occasion;
typedef enum BeamFailureRecoveryConfig__beamFailureRecoveryTimer {
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms10	= 0,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms20	= 1,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms40	= 2,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms60	= 3,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms80	= 4,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms100	= 5,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms150	= 6,
	BeamFailureRecoveryConfig__beamFailureRecoveryTimer_ms200	= 7
} e_BeamFailureRecoveryConfig__beamFailureRecoveryTimer;
typedef enum BeamFailureRecoveryConfig__ext3__spCell_BFR_CBRA_r16 {
	BeamFailureRecoveryConfig__ext3__spCell_BFR_CBRA_r16_true	= 0
} e_BeamFailureRecoveryConfig__ext3__spCell_BFR_CBRA_r16;

/* Forward declarations */
struct RACH_ConfigGeneric;
struct RA_Prioritization;
struct PRACH_ResourceDedicatedBFR;
struct SetupRelease_CandidateBeamRSListExt_r16;

/* BeamFailureRecoveryConfig */
typedef struct BeamFailureRecoveryConfig {
	long	*rootSequenceIndex_BFR;	/* OPTIONAL */
	struct RACH_ConfigGeneric	*rach_ConfigBFR;	/* OPTIONAL */
	RSRP_Range_t	*rsrp_ThresholdSSB;	/* OPTIONAL */
	struct BeamFailureRecoveryConfig__candidateBeamRSList {
		A_SEQUENCE_OF(struct PRACH_ResourceDedicatedBFR) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *candidateBeamRSList;
	long	*ssb_perRACH_Occasion;	/* OPTIONAL */
	long	*ra_ssb_OccasionMaskIndex;	/* OPTIONAL */
	SearchSpaceId_t	*recoverySearchSpaceId;	/* OPTIONAL */
	struct RA_Prioritization	*ra_Prioritization;	/* OPTIONAL */
	long	*beamFailureRecoveryTimer;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct BeamFailureRecoveryConfig__ext1 {
		SubcarrierSpacing_t	*msg1_SubcarrierSpacing;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct BeamFailureRecoveryConfig__ext2 {
		struct RA_Prioritization	*ra_PrioritizationTwoStep_r16;	/* OPTIONAL */
		struct SetupRelease_CandidateBeamRSListExt_r16	*candidateBeamRSListExt_v1610;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct BeamFailureRecoveryConfig__ext3 {
		long	*spCell_BFR_CBRA_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BeamFailureRecoveryConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ssb_perRACH_Occasion_7;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_beamFailureRecoveryTimer_19;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_spCell_BFR_CBRA_r16_35;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_BeamFailureRecoveryConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_BeamFailureRecoveryConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_BeamFailureRecoveryConfig_1[12];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RACH-ConfigGeneric.h"
#include "RA-Prioritization.h"
#include "PRACH-ResourceDedicatedBFR.h"
#include "SetupRelease.h"

#endif	/* _BeamFailureRecoveryConfig_H_ */
#include <asn_internal.h>
