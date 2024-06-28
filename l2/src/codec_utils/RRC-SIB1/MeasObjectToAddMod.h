/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasObjectToAddMod_H_
#define	_MeasObjectToAddMod_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MeasObjectId.h"
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasObjectToAddMod__measObject_PR {
	MeasObjectToAddMod__measObject_PR_NOTHING,	/* No components present */
	MeasObjectToAddMod__measObject_PR_measObjectNR,
	/* Extensions may appear below */
	MeasObjectToAddMod__measObject_PR_measObjectEUTRA,
	MeasObjectToAddMod__measObject_PR_measObjectUTRA_FDD_r16,
	MeasObjectToAddMod__measObject_PR_measObjectNR_SL_r16,
	MeasObjectToAddMod__measObject_PR_measObjectCLI_r16,
	MeasObjectToAddMod__measObject_PR_measObjectRxTxDiff_r17,
	MeasObjectToAddMod__measObject_PR_measObjectRelay_r17
} MeasObjectToAddMod__measObject_PR;

/* Forward declarations */
struct MeasObjectNR;
struct MeasObjectEUTRA;
struct MeasObjectUTRA_FDD_r16;
struct MeasObjectNR_SL_r16;
struct MeasObjectCLI_r16;
struct MeasObjectRxTxDiff_r17;
struct SL_MeasObject_r16;

/* MeasObjectToAddMod */
typedef struct MeasObjectToAddMod {
	MeasObjectId_t	 measObjectId;
	struct MeasObjectToAddMod__measObject {
		MeasObjectToAddMod__measObject_PR present;
		union MeasObjectToAddMod__measObject_u {
			struct MeasObjectNR	*measObjectNR;
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
			struct MeasObjectEUTRA	*measObjectEUTRA;
			struct MeasObjectUTRA_FDD_r16	*measObjectUTRA_FDD_r16;
			struct MeasObjectNR_SL_r16	*measObjectNR_SL_r16;
			struct MeasObjectCLI_r16	*measObjectCLI_r16;
			struct MeasObjectRxTxDiff_r17	*measObjectRxTxDiff_r17;
			struct SL_MeasObject_r16	*measObjectRelay_r17;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} measObject;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasObjectToAddMod_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasObjectToAddMod;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasObjectToAddMod_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasObjectToAddMod_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasObjectNR.h"
#include "MeasObjectEUTRA.h"
#include "MeasObjectUTRA-FDD-r16.h"
#include "MeasObjectNR-SL-r16.h"
#include "MeasObjectCLI-r16.h"
#include "MeasObjectRxTxDiff-r17.h"
#include "SL-MeasObject-r16.h"

#endif	/* _MeasObjectToAddMod_H_ */
#include <asn_internal.h>
