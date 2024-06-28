/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SRS_PeriodicityAndOffsetExt_r16_H_
#define	_SRS_PeriodicityAndOffsetExt_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SRS_PeriodicityAndOffsetExt_r16_PR {
	SRS_PeriodicityAndOffsetExt_r16_PR_NOTHING,	/* No components present */
	SRS_PeriodicityAndOffsetExt_r16_PR_sl128,
	SRS_PeriodicityAndOffsetExt_r16_PR_sl256,
	SRS_PeriodicityAndOffsetExt_r16_PR_sl512,
	SRS_PeriodicityAndOffsetExt_r16_PR_sl20480
} SRS_PeriodicityAndOffsetExt_r16_PR;

/* SRS-PeriodicityAndOffsetExt-r16 */
typedef struct SRS_PeriodicityAndOffsetExt_r16 {
	SRS_PeriodicityAndOffsetExt_r16_PR present;
	union SRS_PeriodicityAndOffsetExt_r16_u {
		long	 sl128;
		long	 sl256;
		long	 sl512;
		long	 sl20480;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SRS_PeriodicityAndOffsetExt_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SRS_PeriodicityAndOffsetExt_r16;
extern asn_CHOICE_specifics_t asn_SPC_SRS_PeriodicityAndOffsetExt_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SRS_PeriodicityAndOffsetExt_r16_1[4];
extern asn_per_constraints_t asn_PER_type_SRS_PeriodicityAndOffsetExt_r16_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _SRS_PeriodicityAndOffsetExt_r16_H_ */
#include <asn_internal.h>
